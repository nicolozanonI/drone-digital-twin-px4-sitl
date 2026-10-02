#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "px4_forwarder/msg/custom_vehicle_odometry.hpp"
#include "px4_forwarder/msg/custom_vehicle_status.hpp"
#include "px4_forwarder/srv/set_float64.hpp"
#include "px4_forwarder/srv/set_network_probabilities.hpp"

#include <random>
#include <chrono>
#include <vector>
#include <deque>
#include <fstream>
#include <sstream>
#include <experimental/filesystem>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <regex>
#include <cmath>
#include <limits>

namespace fs = std::experimental::filesystem;
using namespace std::chrono_literals;

class Forwarder : public rclcpp::Node
{
public:
    Forwarder() : Node("forwarder_node")
    {
        this->declare_parameter("drone_id", "px4_0");
        drone_id_ = this->get_parameter("drone_id").as_string();

        this->declare_parameter("offset_x", 0.0);
        this->declare_parameter("offset_y", 0.0);
        this->declare_parameter("enable_offset_correction", true);
        this->declare_parameter("offsets_yaml_path", "/tmp/px4_drone_offsets.yaml");

        enable_offset_ = this->get_parameter("enable_offset_correction").as_bool();

        load_offsets();

        RCLCPP_INFO(
            this->get_logger(),
            "Offset applicato: X=%.3f, Y=%.3f, enable=%s, source=%s",
            offset_x_,
            offset_y_,
            enable_offset_ ? "true" : "false",
            offset_source_.c_str()
        );

        custom_sequence_odom_ = 200;
        custom_sequence_status_ = 200;

        drop_probability_ = 0.0;
        command_probability_ = 1.0;
        rate_ = 10.0;
        forward_delay_ = 0;

        /*
         * Nuova logica probabilistica:
         *
         * - drop_probability_        indipendente
         * - packet_invalidity_prob_  indipendente
         * - out_of_sequence_prob_    indipendente
         * - delayed_prob_            indipendente
         *
         * packet_invalidity_prob_ controlla globalmente:
         * - corrupt_header
         * - corrupt_data
         * - invalid_timestamp
         *
         * Se packet_invalidity_prob_ = 0.20, allora nel 20% dei messaggi
         * capita UNA tra queste tre anomalie.
         */
        packet_invalidity_prob_ = 0.0;
        out_of_sequence_prob_ = 0.0;
        delayed_prob_ = 0.0;

        rng_ = std::mt19937(std::random_device{}());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();

        odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/" + drone_id_ + "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&Forwarder::odometry_callback, this, std::placeholders::_1)
        );

        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/" + drone_id_ + "/fmu/out/vehicle_status_v1",
            qos,
            std::bind(&Forwarder::status_callback, this, std::placeholders::_1)
        );

        odometry_pub_ = this->create_publisher<px4_forwarder::msg::CustomVehicleOdometry>(
            "/" + drone_id_ + "/forwarder/vehicle/odometry",
            10
        );

        status_pub_ = this->create_publisher<px4_forwarder::msg::CustomVehicleStatus>(
            "/" + drone_id_ + "/forwarder/vehicle/status",
            10
        );

        odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
            std::bind(&Forwarder::odometry_forward_callback, this)
        );

        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
            std::bind(&Forwarder::status_forward_callback, this)
        );

        delivery_timer_ = this->create_wall_timer(
            10ms,
            std::bind(&Forwarder::delivery_callback, this)
        );

        rate_change_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            drone_id_ + "/forwarder/set_forwarder_sending_rate",
            std::bind(
                &Forwarder::set_forwarder_rate_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        fw_prob_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            drone_id_ + "/forwarder/set_forwarder_probability",
            std::bind(
                &Forwarder::set_forwarder_probability_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        command_prob_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            "/" + drone_id_ + "/forwarder/set_command_drop_probability",
            std::bind(
                &Forwarder::set_command_probability_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        /*
         * Nuovi servizi indipendenti.
         *
         * packet_invalidity_probability:
         *   controlla globalmente corrupt header / corrupt data / invalid timestamp.
         *
         * reorder_probability:
         *   controlla out-of-sequence/reorder.
         *
         * packet_delay_probability:
         *   controlla la modifica del timestamp per simulare ritardo.
         */
        packet_invalidity_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            drone_id_ + "/forwarder/set_packet_invalidity_probability",
            std::bind(
                &Forwarder::set_packet_invalidity_probability_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        reorder_prob_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            drone_id_ + "/forwarder/set_reorder_probability",
            std::bind(
                &Forwarder::set_reorder_probability_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        network_probabilities_srv_ =
    this->create_service<px4_forwarder::srv::SetNetworkProbabilities>(
        drone_id_ + "/forwarder/set_network_probabilities",
        std::bind(
            &Forwarder::set_network_probabilities_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

        delayed_prob_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            drone_id_ + "/forwarder/set_packet_delay_probability",
            std::bind(
                &Forwarder::set_packet_delay_probability_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        /*
         * Servizio legacy mantenuto per compatibilità.
         * Ora modifica SOLO packet_invalidity_prob_.
         * Non cambia reorder e non cambia delay.
         */
        packet_issue_legacy_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            drone_id_ + "/forwarder/set_packet_problems_probability",
            std::bind(
                &Forwarder::set_packet_invalidity_probability_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        forwarder_delay_srv_ = this->create_service<px4_forwarder::srv::SetFloat64>(
            drone_id_ + "/forwarder/set_forwarder_messages_delay",
            std::bind(
                &Forwarder::set_forwarder_delay_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        this->declare_parameter("enable_csv_logging", true);
        this->declare_parameter("csv_start_time", "");

        bool enable_csv_logging =
            this->get_parameter("enable_csv_logging").as_bool();

        csv_start_time_ =
            this->get_parameter("csv_start_time").as_string();

        if (enable_csv_logging) {
            if (!configure_csv_start_time(csv_start_time_)) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "CSV logging disabled because csv_start_time is invalid"
                );
            } else {
                init_csv_logging();

                csv_timer_ = this->create_wall_timer(
                    10ms,
                    std::bind(&Forwarder::csv_logging_callback, this)
                );

                if (csv_start_time_enabled_) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "CSV logging enabled, sampling will start at %s",
                        csv_start_time_.c_str()
                    );
                } else {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "CSV logging enabled, sampling starts immediately"
                    );
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "CSV logging disabilitato");
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Forwarder started for %s, odometry + status",
            drone_id_.c_str()
        );
    }

    ~Forwarder()
    {
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file chiuso");
        }
    }

private:
    std::ofstream csv_file_;
    bool csv_file_open_ = false;
    std::string csv_filepath_;
    rclcpp::TimerBase::SharedPtr csv_timer_;
    std::string csv_start_time_;
    bool csv_start_time_enabled_{false};
    bool csv_sampling_started_{false};
    std::chrono::system_clock::time_point csv_start_tp_;

    std::string drone_id_;
    double offset_x_{0.0};
    double offset_y_{0.0};
    bool enable_offset_{true};
    std::string offset_source_{"none"};

    uint32_t custom_sequence_odom_;
    uint32_t custom_sequence_status_;

    double drop_probability_;
    double command_probability_;
    double rate_;
    int forward_delay_;

    double packet_invalidity_prob_;
    double out_of_sequence_prob_;
    double delayed_prob_;

    px4_msgs::msg::VehicleOdometry last_odom_msg_;
    px4_msgs::msg::VehicleStatus last_status_msg_;
    bool has_received_odometry_{false};
    bool has_received_status_{false};

    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_{0.0, 1.0};

    struct DelayedOdometry {
        rclcpp::Time release_time;
        px4_forwarder::msg::CustomVehicleOdometry msg;
    };

    struct DelayedStatus {
        rclcpp::Time release_time;
        px4_forwarder::msg::CustomVehicleStatus msg;
    };

    std::deque<DelayedOdometry> delayed_odom_buffer_;
    std::deque<DelayedStatus> delayed_status_buffer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<px4_forwarder::msg::CustomVehicleOdometry>::SharedPtr odometry_pub_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<px4_forwarder::msg::CustomVehicleStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    rclcpp::TimerBase::SharedPtr delivery_timer_;

    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr rate_change_srv_;
    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr fw_prob_srv_;
    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr command_prob_srv_;
    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr packet_invalidity_srv_;
    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr packet_issue_legacy_srv_;
    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr reorder_prob_srv_;
    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr delayed_prob_srv_;
    rclcpp::Service<px4_forwarder::srv::SetFloat64>::SharedPtr forwarder_delay_srv_;
    rclcpp::Service<px4_forwarder::srv::SetNetworkProbabilities>::SharedPtr network_probabilities_srv_;

    std::string extract_drone_number()
    {
        try {
            size_t pos = drone_id_.find_last_of('_');
            if (pos != std::string::npos) {
                return drone_id_.substr(pos + 1);
            }
        } catch (...) {}

        return "1";
    }

    bool parse_offsets_from_yaml(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }

        std::string line;
        std::string drone_key = "  drone_" + extract_drone_number() + ":";

        bool found_drone = false;
        bool found_x = false;
        bool found_y = false;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }

            if (line.find(drone_key) != std::string::npos) {
                found_drone = true;
                continue;
            }

            if (found_drone) {
                if (line.find("  drone_") != std::string::npos && line.back() == ':') {
                    break;
                }

                if (!found_x && line.find("x:") != std::string::npos) {
                    std::regex rx(R"(x:\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?))");
                    std::smatch match;

                    if (std::regex_search(line, match, rx) && match.size() > 1) {
                        try {
                            offset_x_ = std::stod(match[1].str());
                            found_x = true;
                        } catch (...) {}
                    }
                } else if (!found_y && line.find("y:") != std::string::npos) {
                    std::regex ry(R"(y:\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?))");
                    std::smatch match;

                    if (std::regex_search(line, match, ry) && match.size() > 1) {
                        try {
                            offset_y_ = std::stod(match[1].str());
                            found_y = true;
                        } catch (...) {}
                    }
                }

                if (found_x && found_y) {
                    offset_source_ = "yaml";
                    return true;
                }
            }
        }

        return false;
    }

    void load_offsets()
    {
        std::string yaml_path = this->get_parameter("offsets_yaml_path").as_string();

        if (fs::exists(yaml_path) && parse_offsets_from_yaml(yaml_path)) {
            RCLCPP_INFO(this->get_logger(), "Offset caricati da YAML: %s", yaml_path.c_str());
            return;
        }

        std::string num = extract_drone_number();

        const char* ex = std::getenv(("DRONE_" + num + "_OFFSET_X").c_str());
        const char* ey = std::getenv(("DRONE_" + num + "_OFFSET_Y").c_str());

        if (ex && ey) {
            try {
                offset_x_ = std::stod(ex);
                offset_y_ = std::stod(ey);
                offset_source_ = "env";
                return;
            } catch (...) {}
        }

        offset_x_ = this->get_parameter("offset_x").as_double();
        offset_y_ = this->get_parameter("offset_y").as_double();

        if (offset_x_ != 0.0 || offset_y_ != 0.0) {
            offset_source_ = "params";
        } else {
            offset_source_ = "default";
            RCLCPP_WARN(this->get_logger(), "Nessun offset trovato, uso 0,0");
        }
    }

    void apply_offset_to_position(std::array<float, 3>& pos)
    {
        if (enable_offset_) {
            pos[0] += static_cast<float>(offset_x_);
            pos[1] += static_cast<float>(offset_y_);
        }
    }

    bool configure_csv_start_time(const std::string& time_str)
    {
        if (time_str.empty()) {
            csv_start_time_enabled_ = false;
            csv_sampling_started_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "CSV sampling start time not set: sampling starts immediately"
            );

            return true;
        }

        std::regex time_regex(R"(^([0-1][0-9]|2[0-3]):([0-5][0-9]):([0-5][0-9])$)");
        std::smatch match;

        if (!std::regex_match(time_str, match, time_regex)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid csv_start_time='%s'. Expected format HH:MM:SS, example: 17:21:39",
                time_str.c_str()
            );

            return false;
        }

        int hour = std::stoi(match[1].str());
        int minute = std::stoi(match[2].str());
        int second = std::stoi(match[3].str());

        auto now = std::chrono::system_clock::now();
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

        std::tm local_tm;
        localtime_r(&now_time_t, &local_tm);

        local_tm.tm_hour = hour;
        local_tm.tm_min = minute;
        local_tm.tm_sec = second;

        std::time_t target_time_t = std::mktime(&local_tm);

        if (target_time_t == static_cast<std::time_t>(-1)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to parse csv_start_time='%s'",
                time_str.c_str()
            );

            return false;
        }

        csv_start_tp_ = std::chrono::system_clock::from_time_t(target_time_t);

        /*
         * Se l'orario indicato è già passato oggi,
         * uso la prossima occorrenza, cioè domani alla stessa ora.
         */
        if (csv_start_tp_ <= now) {
            csv_start_tp_ += std::chrono::hours(24);

            RCLCPP_WARN(
                this->get_logger(),
                "csv_start_time='%s' is already passed today. Sampling will start tomorrow at the same time.",
                time_str.c_str()
            );
        }

        csv_start_time_enabled_ = true;
        csv_sampling_started_ = false;

        RCLCPP_INFO(
            this->get_logger(),
            "CSV sampling scheduled at local time %02d:%02d:%02d",
            hour,
            minute,
            second
        );

        return true;
    }

    void init_csv_logging()
    {
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
        }

        csv_file_.clear();

        const char* home = std::getenv("HOME");
        if (!home) {
            return;
        }

        fs::path dir = fs::path(home) / "digital_twin_benchmark_datasets";

        if (!fs::exists(dir)) {
            try {
                fs::create_directories(dir);
            } catch (...) {
                return;
            }
        }

        auto now = std::chrono::system_clock::now();
        auto tt = std::chrono::system_clock::to_time_t(now);

        std::stringstream filename;
        filename << "px4_dataset_"
                 << drone_id_
                 << "_"
                 << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S")
                 << ".csv";

        csv_filepath_ = (dir / filename.str()).string();
        csv_file_.open(csv_filepath_, std::ios::out | std::ios::app);

        if (!csv_file_.is_open()) {
            return;
        }

        if (fs::file_size(csv_filepath_) == 0) {
            csv_file_
                << "log_timestamp_us,"
                << "log_ros_sec,log_ros_nsec,"
                << "pos_x,pos_y,pos_z,"
                << "vel_x,vel_y,vel_z"
                << std::endl;
            csv_file_.flush();

            RCLCPP_INFO(this->get_logger(), "CSV creato: %s", csv_filepath_.c_str());
        }

        csv_file_open_ = true;
    }

    void csv_log_odometry(const px4_msgs::msg::VehicleOdometry& msg)
    {
        if (!csv_file_open_ || !has_received_odometry_) {
            return;
        }

        /*
         * Timestamp del momento in cui viene scritta la riga CSV.
         * Questo NON è il timestamp PX4 del messaggio.
         */
        const int64_t log_timestamp_ns = this->now().nanoseconds();

        const uint64_t log_timestamp_us =
            static_cast<uint64_t>(log_timestamp_ns / 1000LL);

        const uint64_t log_ros_sec =
            static_cast<uint64_t>(log_timestamp_ns / 1000000000LL);

        const uint64_t log_ros_nsec =
            static_cast<uint64_t>(log_timestamp_ns % 1000000000LL);

        /*
         * Posizione salvata con offset applicato.
         * Se vuoi la posizione grezza PX4, rimuovi apply_offset_to_position(pos).
         */
        std::array<float, 3> pos = msg.position;
        apply_offset_to_position(pos);

        const auto& vel = msg.velocity;

        csv_file_ << std::fixed
                  << std::setprecision(std::numeric_limits<float>::max_digits10)
                  << log_timestamp_us << ","
                  << log_ros_sec << ","
                  << log_ros_nsec << ","
                  << pos[0] << ","
                  << pos[1] << ","
                  << pos[2] << ","
                  << vel[0] << ","
                  << vel[1] << ","
                  << vel[2]
                  << std::endl;

        static int cnt = 0;
        if (++cnt % 100 == 0) {
            csv_file_.flush();
        }
    }

    void csv_logging_callback()
    {
        if (!csv_sampling_started_) {
            if (csv_start_time_enabled_) {
                auto now = std::chrono::system_clock::now();

                if (now < csv_start_tp_) {
                    return;
                }
            }

            csv_sampling_started_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "CSV sampling started"
            );
        }

        if (has_received_odometry_) {
            csv_log_odometry(last_odom_msg_);
        }
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        last_odom_msg_ = *msg;
        has_received_odometry_ = true;
    }

    void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        last_status_msg_ = *msg;
        has_received_status_ = true;
    }

    void delivery_callback()
    {
        auto now = this->now();

        while (!delayed_odom_buffer_.empty() &&
               delayed_odom_buffer_.front().release_time <= now) {
            odometry_pub_->publish(delayed_odom_buffer_.front().msg);
            delayed_odom_buffer_.pop_front();
        }

        while (!delayed_status_buffer_.empty() &&
               delayed_status_buffer_.front().release_time <= now) {
            status_pub_->publish(delayed_status_buffer_.front().msg);
            delayed_status_buffer_.pop_front();
        }
    }

    std::string corrupt_header_odom(px4_forwarder::msg::CustomVehicleOdometry& msg)
    {
        std::vector<std::string> opts = {
            "empty_frames",
            "max_seq",
            "invalid_ts_sample"
        };

        std::uniform_int_distribution<> d(0, static_cast<int>(opts.size()) - 1);
        std::string t = opts[d(rng_)];

        if (t == "empty_frames") {
            msg.pose_frame = 0;
            msg.velocity_frame = 0;
            return "CORRUPT_HEADER(empty_frames)";
        }

        if (t == "max_seq") {
            msg.seq = 4294967295u;
            return "CORRUPT_HEADER(max_seq)";
        }

        std::uniform_int_distribution<> tsd(1000, 2000);
        msg.timestamp_sample = tsd(rng_);
        return "CORRUPT_HEADER(invalid_ts_sample)";
    }

    std::string corrupt_header_status(px4_forwarder::msg::CustomVehicleStatus& msg)
    {
        std::vector<std::string> opts = {
            "invalid_nav_state",
            "invalid_arming_state",
            "max_seq"
        };

        std::uniform_int_distribution<> d(0, static_cast<int>(opts.size()) - 1);
        std::string t = opts[d(rng_)];

        if (t == "invalid_nav_state") {
            msg.nav_state = 255;
            return "CORRUPT_HEADER(invalid_nav_state)";
        }

        if (t == "invalid_arming_state") {
            msg.arming_state = 255;
            return "CORRUPT_HEADER(invalid_arming_state)";
        }

        msg.seq = 4294967295u;
        return "CORRUPT_HEADER(max_seq)";
    }

    std::string corrupt_data_odom(px4_forwarder::msg::CustomVehicleOdometry& msg)
    {
        std::vector<std::string> opts = {
            "nan_position",
            "huge_position",
            "nan_velocity",
            "huge_velocity",
            "bad_quaternion"
        };

        std::uniform_int_distribution<> d(0, static_cast<int>(opts.size()) - 1);
        std::string t = opts[d(rng_)];

        if (t == "nan_position") {
            msg.position[0] = std::numeric_limits<float>::quiet_NaN();
            return "CORRUPT_DATA(nan_position)";
        }

        if (t == "huge_position") {
            msg.position[0] = 1e9f;
            msg.position[1] = -1e9f;
            return "CORRUPT_DATA(huge_position)";
        }

        if (t == "nan_velocity") {
            msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
            return "CORRUPT_DATA(nan_velocity)";
        }

        if (t == "huge_velocity") {
            msg.velocity[0] = 1e6f;
            msg.velocity[1] = -1e6f;
            msg.velocity[2] = 1e6f;
            return "CORRUPT_DATA(huge_velocity)";
        }

        msg.q[0] = 0.0f;
        msg.q[1] = 0.0f;
        msg.q[2] = 0.0f;
        msg.q[3] = 0.0f;
        return "CORRUPT_DATA(bad_quaternion)";
    }

    std::string corrupt_data_status(px4_forwarder::msg::CustomVehicleStatus& msg)
    {
        std::vector<std::string> opts = {
            "invalid_vehicle_type",
            "invalid_hil_state",
            "invalid_nav_state",
            "invalid_arming_state"
        };

        std::uniform_int_distribution<> d(0, static_cast<int>(opts.size()) - 1);
        std::string t = opts[d(rng_)];

        if (t == "invalid_vehicle_type") {
            msg.vehicle_type = 255;
            return "CORRUPT_DATA(invalid_vehicle_type)";
        }

        if (t == "invalid_hil_state") {
            msg.hil_state = 255;
            return "CORRUPT_DATA(invalid_hil_state)";
        }

        if (t == "invalid_nav_state") {
            msg.nav_state = 255;
            return "CORRUPT_DATA(invalid_nav_state)";
        }

        msg.arming_state = 255;
        return "CORRUPT_DATA(invalid_arming_state)";
    }

    std::string make_delayed_odom(px4_forwarder::msg::CustomVehicleOdometry& msg)
    {
        double ms;

        if (forward_delay_ > 0) {
            ms = static_cast<double>(forward_delay_);
        } else {
            std::uniform_real_distribution<> d(1000.0, 5000.0);
            ms = d(rng_);
        }

        auto release_time =
            this->now() + rclcpp::Duration::from_seconds(ms / 1000.0);

        delayed_odom_buffer_.push_back({release_time, msg});

        char buf[100];
        snprintf(buf, sizeof(buf), "TRUE_DELAY(%.0fms)", ms);
        return std::string(buf);
    }

    std::string make_delayed_status(px4_forwarder::msg::CustomVehicleStatus& msg)
    {
        double ms;

        if (forward_delay_ > 0) {
            ms = static_cast<double>(forward_delay_);
        } else {
            std::uniform_real_distribution<> d(1000.0, 5000.0);
            ms = d(rng_);
        }

        auto release_time =
            this->now() + rclcpp::Duration::from_seconds(ms / 1000.0);

        delayed_status_buffer_.push_back({release_time, msg});

        char buf[100];
        snprintf(buf, sizeof(buf), "TRUE_DELAY(%.0fms)", ms);
        return std::string(buf);
    }

    std::string corrupt_timestamp_odom(px4_forwarder::msg::CustomVehicleOdometry& msg)
    {
        std::vector<std::string> opts = {
            "future",
            "year2000",
            "year3000"
        };

        std::uniform_int_distribution<> d(0, static_cast<int>(opts.size()) - 1);
        std::string t = opts[d(rng_)];

        if (t == "future") {
            auto ft = this->now() + rclcpp::Duration(10000, 0);
            msg.timestamp = static_cast<uint64_t>(ft.nanoseconds() / 1000);
            return "CORRUPT_TS(future)";
        }

        if (t == "year2000") {
            msg.timestamp = 946684800000000ULL;
            return "CORRUPT_TS(year2000)";
        }

        msg.timestamp = 32503680000000000ULL;
        return "CORRUPT_TS(year3000)";
    }

    std::string corrupt_timestamp_status(px4_forwarder::msg::CustomVehicleStatus& msg)
    {
        std::vector<std::string> opts = {
            "future",
            "year2000",
            "year3000"
        };

        std::uniform_int_distribution<> d(0, static_cast<int>(opts.size()) - 1);
        std::string t = opts[d(rng_)];

        if (t == "future") {
            auto ft = this->now() + rclcpp::Duration(10000, 0);
            msg.timestamp = static_cast<uint64_t>(ft.nanoseconds() / 1000);
            return "CORRUPT_TS(future)";
        }

        if (t == "year2000") {
            msg.timestamp = 946684800000000ULL;
            return "CORRUPT_TS(year2000)";
        }

        msg.timestamp = 32503680000000000ULL;
        return "CORRUPT_TS(year3000)";
    }

    void make_out_of_sequence_odom(px4_forwarder::msg::CustomVehicleOdometry& msg)
    {
        std::uniform_real_distribution<> d(1.0, 2.0);
        auto release_time = this->now() + rclcpp::Duration::from_seconds(d(rng_));
        delayed_odom_buffer_.push_back({release_time, msg});
    }

    void make_out_of_sequence_status(px4_forwarder::msg::CustomVehicleStatus& msg)
    {
        std::uniform_real_distribution<> d(1.0, 2.0);
        auto release_time = this->now() + rclcpp::Duration::from_seconds(d(rng_));
        delayed_status_buffer_.push_back({release_time, msg});
    }

    std::string apply_packet_invalidity_odom(
        px4_forwarder::msg::CustomVehicleOdometry& msg
    )
    {
        std::uniform_int_distribution<> d(0, 2);
        int choice = d(rng_);

        if (choice == 0) {
            return corrupt_header_odom(msg);
        }

        if (choice == 1) {
            return corrupt_data_odom(msg);
        }

        return corrupt_timestamp_odom(msg);
    }

    std::string apply_packet_invalidity_status(
        px4_forwarder::msg::CustomVehicleStatus& msg
    )
    {
        std::uniform_int_distribution<> d(0, 2);
        int choice = d(rng_);

        if (choice == 0) {
            return corrupt_header_status(msg);
        }

        if (choice == 1) {
            return corrupt_data_status(msg);
        }

        return corrupt_timestamp_status(msg);
    }

    void odometry_forward_callback()
    {
        custom_sequence_odom_++;

        if (!has_received_odometry_) {
            return;
        }

        px4_forwarder::msg::CustomVehicleOdometry fwd;

        fwd.seq = custom_sequence_odom_;
        fwd.timestamp = last_odom_msg_.timestamp;
        fwd.timestamp_sample = last_odom_msg_.timestamp_sample;

        fwd.position = last_odom_msg_.position;
        apply_offset_to_position(fwd.position);

        fwd.q = last_odom_msg_.q;
        fwd.velocity = last_odom_msg_.velocity;
        fwd.angular_velocity = last_odom_msg_.angular_velocity;
        fwd.position_variance = last_odom_msg_.position_variance;
        fwd.orientation_variance = last_odom_msg_.orientation_variance;
        fwd.velocity_variance = last_odom_msg_.velocity_variance;
        fwd.pose_frame = last_odom_msg_.pose_frame;
        fwd.velocity_frame = last_odom_msg_.velocity_frame;

        std::vector<std::string> errors;

        /*
         * Logica cumulativa tra:
         *
         * - drop_probability_
         * - packet_invalidity_prob_
         * - out_of_sequence_prob_
         *
         * Esempio:
         * drop=0.20, invalidity=0.30, out_of_order=0.25
         *
         * 0.00 - 0.20 -> DROP
         * 0.20 - 0.50 -> PACKET INVALIDITY
         * 0.50 - 0.75 -> OUT OF ORDER
         * 0.75 - 1.00 -> NORMALE
         */

        const double r = dist_(rng_);
        double threshold = 0.0;

        // DROP
        threshold += drop_probability_;
        if (r < threshold) {
            RCLCPP_INFO(
                this->get_logger(),
                "[ODOM DROP] seq=%u",
                custom_sequence_odom_
            );
            return;
        }

        // PACKET INVALIDITY
        threshold += packet_invalidity_prob_;
        if (r < threshold) {
            std::string invalidity_error = apply_packet_invalidity_odom(fwd);

            if (!invalidity_error.empty()) {
                errors.push_back(invalidity_error);
            }

            odometry_pub_->publish(fwd);
        }
        // OUT OF ORDER
        else {
            threshold += out_of_sequence_prob_;

            if (r < threshold) {
                make_out_of_sequence_odom(fwd);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[REORDER] odom seq=%u queued out-of-sequence",
                    fwd.seq
                );

                return;
            }

            // TRUE DELAY
            threshold += delayed_prob_;

            if (r < threshold) {
                std::string delay_error = make_delayed_odom(fwd);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[%s] odom seq=%u queued delayed",
                    delay_error.c_str(),
                    fwd.seq
                );

                return;
            }

            // NORMAL
            odometry_pub_->publish(fwd);
        }

        if (!errors.empty()) {
            std::stringstream ss;

            for (size_t i = 0; i < errors.size(); ++i) {
                if (i > 0) {
                    ss << " + ";
                }
                ss << errors[i];
            }

            RCLCPP_INFO(
                this->get_logger(),
                "[%s] odom seq=%u",
                ss.str().c_str(),
                fwd.seq
            );
        }
    }

    void status_forward_callback()
    {
        custom_sequence_status_++;

        if (!has_received_status_) {
            return;
        }

        px4_forwarder::msg::CustomVehicleStatus fwd;

        fwd.seq = custom_sequence_status_;
        fwd.timestamp = last_status_msg_.timestamp;
        fwd.arming_state = last_status_msg_.arming_state;
        fwd.nav_state = last_status_msg_.nav_state;
        fwd.failsafe = last_status_msg_.failsafe;
        fwd.pre_flight_checks_pass = last_status_msg_.pre_flight_checks_pass;
        fwd.safety_off = last_status_msg_.safety_off;
        fwd.hil_state = last_status_msg_.hil_state;
        fwd.vehicle_type = last_status_msg_.vehicle_type;
        fwd.gcs_connection_lost = last_status_msg_.gcs_connection_lost;
        fwd.in_transition_mode = last_status_msg_.in_transition_mode;

        std::vector<std::string> errors;

        /*
         * Logica cumulativa tra:
         *
         * - drop_probability_
         * - packet_invalidity_prob_
         * - out_of_sequence_prob_
         */

        const double r = dist_(rng_);
        double threshold = 0.0;

        // DROP
        threshold += drop_probability_;
        if (r < threshold) {
            RCLCPP_INFO(
                this->get_logger(),
                "[STATUS DROP] seq=%u",
                custom_sequence_status_
            );
            return;
        }

        // PACKET INVALIDITY
        threshold += packet_invalidity_prob_;
        if (r < threshold) {
            std::string invalidity_error = apply_packet_invalidity_status(fwd);

            if (!invalidity_error.empty()) {
                errors.push_back(invalidity_error);
            }

            status_pub_->publish(fwd);
        }
        // OUT OF ORDER
        else {
            threshold += out_of_sequence_prob_;

            if (r < threshold) {
                make_out_of_sequence_status(fwd);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[REORDER] status seq=%u queued out-of-sequence",
                    fwd.seq
                );

                return;
            }

            // TRUE DELAY
            threshold += delayed_prob_;

            if (r < threshold) {
                std::string delay_error = make_delayed_status(fwd);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[%s] status seq=%u queued delayed",
                    delay_error.c_str(),
                    fwd.seq
                );

                return;
            }

            // NORMAL
            status_pub_->publish(fwd);
        }

        if (!errors.empty()) {
            std::stringstream ss;

            for (size_t i = 0; i < errors.size(); ++i) {
                if (i > 0) {
                    ss << " + ";
                }
                ss << errors[i];
            }

            RCLCPP_INFO(
                this->get_logger(),
                "[%s] status seq=%u nav_state=%u",
                ss.str().c_str(),
                fwd.seq,
                fwd.nav_state
            );
        }
    }

    bool validate_network_probabilities(
        double drop,
        double packet_invalidity,
        double out_of_order,
        double delay,
        std::string& error_message)
    {
        if (drop < 0.0 || drop > 1.0) {
            error_message = "drop_probability must be in range [0.0, 1.0]";
            return false;
        }

        if (packet_invalidity < 0.0 || packet_invalidity > 1.0) {
            error_message = "packet_invalidity_probability must be in range [0.0, 1.0]";
            return false;
        }

        if (out_of_order < 0.0 || out_of_order > 1.0) {
            error_message = "out_of_order_probability must be in range [0.0, 1.0]";
            return false;
        }

        if (delay < 0.0 || delay > 1.0) {
            error_message = "delay_probability must be in range [0.0, 1.0]";
            return false;
        }

        const double sum = drop + packet_invalidity + out_of_order + delay;

        if (sum > 1.0) {
            std::stringstream ss;
            ss << "drop + packet_invalidity + out_of_order + delay must be <= 1.0. Current sum="
               << sum;
            error_message = ss.str();
            return false;
        }

        return true;
    }

    bool set_probability_value(
        double value,
        double& target,
        const std::string& name,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        if (value < 0.0 || value > 1.0) {
            resp->success = false;
            resp->message = name + " must be in range [0.0, 1.0]";
            return false;
        }

        target = value;

        resp->success = true;
        resp->message = name + " updated";

        RCLCPP_INFO(
            this->get_logger(),
            "%s updated to %.3f",
            name.c_str(),
            value
        );

        return true;
    }

    void set_forwarder_rate_callback(
        const std::shared_ptr<px4_forwarder::srv::SetFloat64::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        if (req->value > 0.0) {
            rate_ = req->value;

            odom_timer_->cancel();
            status_timer_->cancel();

            odom_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
                std::bind(&Forwarder::odometry_forward_callback, this)
            );

            status_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
                std::bind(&Forwarder::status_forward_callback, this)
            );

            resp->success = true;
            resp->message = "Rate updated";

            RCLCPP_INFO(
                this->get_logger(),
                "Rate updated to %.2f Hz",
                rate_
            );
        } else {
            resp->success = false;
            resp->message = "Only positive values are valid";
        }
    }

    void set_forwarder_probability_callback(
        const std::shared_ptr<px4_forwarder::srv::SetFloat64::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        std::string error_message;

        if (!validate_network_probabilities(
                 req->value,
                 packet_invalidity_prob_,
                 out_of_sequence_prob_,
                 delayed_prob_,
                 error_message)) {
            resp->success = false;
            resp->message = error_message;
            return;
        }

        drop_probability_ = req->value;

        const double normal_probability =
            1.0 - drop_probability_ - packet_invalidity_prob_ - out_of_sequence_prob_ - delayed_prob_;
        resp->success = true;
        resp->message = "drop_probability updated";

        RCLCPP_INFO(
            this->get_logger(),
            "Network cumulative probabilities: drop=%.3f, packet_invalidity=%.3f, out_of_order=%.3f, delay=%.3f, normal=%.3f",
            drop_probability_,
            packet_invalidity_prob_,
            out_of_sequence_prob_,
            delayed_prob_,
            normal_probability
        );
    }

    void set_command_probability_callback(
        const std::shared_ptr<px4_forwarder::srv::SetFloat64::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        set_probability_value(
            req->value,
            command_probability_,
            "command_probability",
            resp
        );
    }

    void set_packet_invalidity_probability_callback(
        const std::shared_ptr<px4_forwarder::srv::SetFloat64::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        std::string error_message;

        if (!validate_network_probabilities(
                drop_probability_,
                req->value,
                out_of_sequence_prob_,
                delayed_prob_,
                error_message)) {
            resp->success = false;
            resp->message = error_message;
            return;
        }

        packet_invalidity_prob_ = req->value;

        const double normal_probability =
            1.0 - drop_probability_ - packet_invalidity_prob_ - out_of_sequence_prob_ - delayed_prob_;

        resp->success = true;
        resp->message = "packet_invalidity_probability updated";

        RCLCPP_INFO(
            this->get_logger(),
            "Network cumulative probabilities: drop=%.3f, packet_invalidity=%.3f, out_of_order=%.3f, delay=%.3f, normal=%.3f",
            drop_probability_,
            packet_invalidity_prob_,
            out_of_sequence_prob_,
            delayed_prob_,
            normal_probability
        );
    }

    void set_reorder_probability_callback(
        const std::shared_ptr<px4_forwarder::srv::SetFloat64::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        std::string error_message;

        if (!validate_network_probabilities(
                drop_probability_,
                packet_invalidity_prob_,
                req->value,
                delayed_prob_,
                error_message)) {
            resp->success = false;
            resp->message = error_message;
            return;
        }

        out_of_sequence_prob_ = req->value;

        const double normal_probability =
            1.0 - drop_probability_ - packet_invalidity_prob_ - out_of_sequence_prob_ - delayed_prob_;

        resp->success = true;
        resp->message = "reorder_probability updated";

        RCLCPP_INFO(
            this->get_logger(),
            "Network cumulative probabilities: drop=%.3f, packet_invalidity=%.3f, out_of_order=%.3f, delay=%.3f, normal=%.3f",
            drop_probability_,
            packet_invalidity_prob_,
            out_of_sequence_prob_,
            delayed_prob_,
            normal_probability
        );
    }

    void set_packet_delay_probability_callback(
        const std::shared_ptr<px4_forwarder::srv::SetFloat64::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        std::string error_message;

        if (!validate_network_probabilities(
                drop_probability_,
                packet_invalidity_prob_,
                out_of_sequence_prob_,
                req->value,
                error_message)) {
            resp->success = false;
            resp->message = error_message;
            return;
        }

        delayed_prob_ = req->value;

        const double normal_probability =
            1.0 - drop_probability_ - packet_invalidity_prob_ - out_of_sequence_prob_ - delayed_prob_;

        resp->success = true;
        resp->message = "packet_delay_probability updated";

        RCLCPP_INFO(
            this->get_logger(),
            "Network cumulative probabilities: drop=%.3f, packet_invalidity=%.3f, out_of_order=%.3f, delay=%.3f, normal=%.3f",
            drop_probability_,
            packet_invalidity_prob_,
            out_of_sequence_prob_,
            delayed_prob_,
            normal_probability
        );
    }

    void set_network_probabilities_callback(
        const std::shared_ptr<px4_forwarder::srv::SetNetworkProbabilities::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetNetworkProbabilities::Response> resp
    )
    {
        std::string error_message;

        if (!validate_network_probabilities(
                req->drop_probability,
                req->packet_invalidity_probability,
                req->out_of_order_probability,
                req->delay_probability,
                error_message)) {
            resp->success = false;
            resp->message = error_message;
            return;
        }

        drop_probability_ = req->drop_probability;
        packet_invalidity_prob_ = req->packet_invalidity_probability;
        out_of_sequence_prob_ = req->out_of_order_probability;
        delayed_prob_ = req->delay_probability;

        const double normal_probability =
            1.0 - drop_probability_ - packet_invalidity_prob_ - out_of_sequence_prob_ - delayed_prob_;

        resp->success = true;

        std::stringstream ss;
        ss << "Network cumulative probabilities updated: "
           << "drop=" << drop_probability_
           << ", packet_invalidity=" << packet_invalidity_prob_
           << ", out_of_order=" << out_of_sequence_prob_
           << ", delay=" << delayed_prob_
           << ", normal=" << normal_probability;

        resp->message = ss.str();

        RCLCPP_INFO(
            this->get_logger(),
            "Network cumulative probabilities updated: drop=%.3f, packet_invalidity=%.3f, out_of_order=%.3f, delay=%.3f, normal=%.3f",
            drop_probability_,
            packet_invalidity_prob_,
            out_of_sequence_prob_,
            delayed_prob_,
            normal_probability
        );
    }

    void set_forwarder_delay_callback(
        const std::shared_ptr<px4_forwarder::srv::SetFloat64::Request> req,
        std::shared_ptr<px4_forwarder::srv::SetFloat64::Response> resp
    )
    {
        if (req->value >= 0.0) {
            forward_delay_ = static_cast<int>(req->value);

            resp->success = true;
            resp->message = "Delay updated";

            RCLCPP_INFO(
                this->get_logger(),
                "Fixed delay updated to %d ms. If 0, random delay is used",
                forward_delay_
            );
        } else {
            resp->success = false;
            resp->message = "Only non-negative values are valid";
        }
    }
};

int main(int argc, char* argv[])
{
    std::cout << "Starting forwarder node, odometry + status..." << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Forwarder>());
    rclcpp::shutdown();

    return 0;
}