#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <px4_forwarder/srv/set_network_probabilities.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <sstream>

using SetBool = std_srvs::srv::SetBool;
using SetNetworkProbabilities = px4_forwarder::srv::SetNetworkProbabilities;
using namespace std::chrono_literals;

/* =========================
 *  Data structures
 * ========================= */

struct ServiceGroup
{
  std::string service_name;
  std::string forwarder_suffix;
  std::vector<std::string> drone_ids;

  double drop_probability{0.2};
  double packet_invalidity_probability{0.2};
  double out_of_order_probability{0.2};
  double delay_probability{0.0};
};

static std::string join_service_name(
  const std::string & a,
  const std::string & b)
{
  if (a.empty()) {
    return b;
  }

  if (b.empty()) {
    return a;
  }

  const bool a_slash = (!a.empty() && a.back() == '/');
  const bool b_slash = (!b.empty() && b.front() == '/');

  if (a_slash && b_slash) {
    return a + b.substr(1);
  }

  if (!a_slash && !b_slash) {
    return a + "/" + b;
  }

  return a + b;
}

/* =========================
 *  Node
 * ========================= */

class ScenarioOrchestratorNode : public rclcpp::Node
{
public:
  ScenarioOrchestratorNode()
  : Node("scenario_orchestrator")
  {
    this->declare_parameter<std::string>("config_file", "");

    const auto config_file =
      this->get_parameter("config_file").as_string();

    if (config_file.empty()) {
      throw std::runtime_error(
        "Parameter 'config_file' is required but not set");
    }

    load_groups_from_yaml(config_file);
    create_services();

    RCLCPP_INFO(
      this->get_logger(),
      "Scenario orchestrator ready (%zu service group(s))",
      service_groups_.size());
  }

private:
  /* =========================
   *  Validation helpers
   * ========================= */

  bool validate_probability(
    double value,
    const std::string & name,
    std::string & error_message)
  {
    if (value < 0.0 || value > 1.0) {
      error_message =
        name + " must be in range [0.0, 1.0]";
      return false;
    }

    return true;
  }

  bool validate_group_probabilities(
    const ServiceGroup & g,
    std::string & error_message)
  {
    if (!validate_probability(
          g.drop_probability,
          "drop_probability",
          error_message)) {
      return false;
    }

    if (!validate_probability(
          g.packet_invalidity_probability,
          "packet_invalidity_probability",
          error_message)) {
      return false;
    }

    if (!validate_probability(
          g.out_of_order_probability,
          "out_of_order_probability",
          error_message)) {
      return false;
    }

    if (!validate_probability(
          g.delay_probability,
          "delay_probability",
          error_message)) {
      return false;
    }

    const double sum =
      g.drop_probability +
      g.packet_invalidity_probability +
      g.out_of_order_probability +
      g.delay_probability;

    if (sum > 1.0) {
      std::stringstream ss;
      ss << "Invalid probabilities for service '"
         << g.service_name
         << "': drop + packet_invalidity + out_of_order + delay "
         << "must be <= 1.0. Current sum="
         << sum;

      error_message = ss.str();
      return false;
    }

    return true;
  }

  /* =========================
   *  YAML loading
   * ========================= */

  void load_groups_from_yaml(const std::string & path)
  {
    YAML::Node root = YAML::LoadFile(path);

    if (!root["services"] || !root["services"].IsSequence()) {
      throw std::runtime_error(
        "YAML file must contain a 'services' list");
    }

    for (const auto & item : root["services"]) {
      ServiceGroup g;

      if (!item["name"]) {
        throw std::runtime_error(
          "Each service entry must contain a 'name'");
      }

      g.service_name =
        item["name"].as<std::string>();

      g.forwarder_suffix =
        item["forwarder_service_suffix"]
          ? item["forwarder_service_suffix"].as<std::string>()
          : "/forwarder/set_network_probabilities";

      if (item["drop_probability"]) {
        g.drop_probability =
          item["drop_probability"].as<double>();
      }

      if (item["packet_invalidity_probability"]) {
        g.packet_invalidity_probability =
          item["packet_invalidity_probability"].as<double>();
      }

      if (item["out_of_order_probability"]) {
        g.out_of_order_probability =
          item["out_of_order_probability"].as<double>();
      }

      if (item["delay_probability"]) {
        g.delay_probability =
          item["delay_probability"].as<double>();
      }

      if (!item["drone_ids"] || !item["drone_ids"].IsSequence()) {
        throw std::runtime_error(
          "Each service entry must contain a 'drone_ids' list");
      }

      for (const auto & d : item["drone_ids"]) {
        g.drone_ids.push_back(d.as<std::string>());
      }

      if (g.drone_ids.empty()) {
        throw std::runtime_error(
          "Each service entry must contain at least one drone_id");
      }

      std::string error_message;
      if (!validate_group_probabilities(g, error_message)) {
        throw std::runtime_error(error_message);
      }

      service_groups_.push_back(g);

      const double normal_probability =
        1.0 -
        g.drop_probability -
        g.packet_invalidity_probability -
        g.out_of_order_probability -
        g.delay_probability;

      RCLCPP_INFO(
        this->get_logger(),
        "Loaded service '%s' (%zu drones): drop=%.3f invalidity=%.3f out_of_order=%.3f delay=%.3f normal=%.3f suffix=%s",
        g.service_name.c_str(),
        g.drone_ids.size(),
        g.drop_probability,
        g.packet_invalidity_probability,
        g.out_of_order_probability,
        g.delay_probability,
        normal_probability,
        g.forwarder_suffix.c_str());
    }
  }

  /* =========================
   *  Service creation
   * ========================= */

  void create_services()
  {
    for (const auto & g : service_groups_) {
      auto srv = this->create_service<SetBool>(
        g.service_name,
        [this, g](
          const std::shared_ptr<SetBool::Request> req,
          std::shared_ptr<SetBool::Response> res)
        {
          handle_group_request(g, req, res);
        });

      services_.push_back(srv);

      RCLCPP_INFO(
        this->get_logger(),
        "Service ready: %s (SetBool)",
        g.service_name.c_str());
    }
  }

  /* =========================
   *  Callback logic
   * ========================= */

  void handle_group_request(
    const ServiceGroup & g,
    const std::shared_ptr<SetBool::Request> req,
    std::shared_ptr<SetBool::Response> res)
  {
    const bool enabled = req->data;

    const double drop_probability =
      enabled ? g.drop_probability : 0.0;

    const double packet_invalidity_probability =
      enabled ? g.packet_invalidity_probability : 0.0;

    const double out_of_order_probability =
      enabled ? g.out_of_order_probability : 0.0;

    const double delay_probability =
      enabled ? g.delay_probability : 0.0;

    const double normal_probability =
      1.0 -
      drop_probability -
      packet_invalidity_probability -
      out_of_order_probability -
      delay_probability;

    size_t dispatched = 0;

    for (const auto & drone_id : g.drone_ids) {
      const std::string forwarder_service =
        join_service_name(drone_id, g.forwarder_suffix);

      auto client = get_or_create_client(forwarder_service);

      if (!client->wait_for_service(200ms)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Forwarder service unavailable: %s",
          forwarder_service.c_str());
        continue;
      }

      auto f_req =
        std::make_shared<SetNetworkProbabilities::Request>();

      f_req->drop_probability =
        drop_probability;

      f_req->packet_invalidity_probability =
        packet_invalidity_probability;

      f_req->out_of_order_probability =
        out_of_order_probability;

      f_req->delay_probability =
        delay_probability;

      client->async_send_request(
        f_req,
        [this, forwarder_service](
          rclcpp::Client<SetNetworkProbabilities>::SharedFuture future)
        {
          try {
            auto response = future.get();

            if (response->success) {
              RCLCPP_INFO(
                this->get_logger(),
                "Forwarder service accepted request: %s -> %s",
                forwarder_service.c_str(),
                response->message.c_str());
            } else {
              RCLCPP_WARN(
                this->get_logger(),
                "Forwarder service rejected request: %s -> %s",
                forwarder_service.c_str(),
                response->message.c_str());
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR(
              this->get_logger(),
              "Failed to process response from %s: %s",
              forwarder_service.c_str(),
              e.what());
          }
        });

      dispatched++;
    }

    res->success = (dispatched > 0);

    std::stringstream ss;
    ss << "enabled=" << (enabled ? "true" : "false")
       << " drop=" << drop_probability
       << " packet_invalidity=" << packet_invalidity_probability
       << " out_of_order=" << out_of_order_probability
       << " delay=" << delay_probability
       << " normal=" << normal_probability
       << " dispatched=" << dispatched
       << "/" << g.drone_ids.size();

    res->message = ss.str();

    RCLCPP_INFO(
      this->get_logger(),
      "%s",
      res->message.c_str());
  }

  /* =========================
   *  Client cache
   * ========================= */

  rclcpp::Client<SetNetworkProbabilities>::SharedPtr
  get_or_create_client(const std::string & service_name)
  {
    auto it = client_cache_.find(service_name);

    if (it != client_cache_.end()) {
      return it->second;
    }

    auto client =
      this->create_client<SetNetworkProbabilities>(service_name);

    client_cache_[service_name] = client;
    return client;
  }

  /* =========================
   *  Members
   * ========================= */

  std::vector<ServiceGroup> service_groups_;

  std::vector<rclcpp::Service<SetBool>::SharedPtr> services_;

  std::unordered_map<
    std::string,
    rclcpp::Client<SetNetworkProbabilities>::SharedPtr> client_cache_;
};

/* =========================
 *  main
 * ========================= */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ScenarioOrchestratorNode>());

  rclcpp::shutdown();

  return 0;
}