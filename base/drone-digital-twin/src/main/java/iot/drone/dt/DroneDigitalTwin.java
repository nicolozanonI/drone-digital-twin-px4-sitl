package iot.drone.dt;

import io.github.twinklekhj.ros.core.RosBridge;
import io.github.twinklekhj.ros.op.RosSubscription;
import io.vertx.core.json.JsonObject;
import iot.drone.dt.adapter.physical.websocket.ros.RosBridgeClient;
import iot.drone.dt.adapter.physical.websocket.ros.WebSocketRosPhysicalAdapter;
import iot.drone.dt.adapter.physical.websocket.ros.WebSocketRosPhysicalAdapterConfiguration;
import iot.drone.dt.adapter.physical.websocket.ros.exception.WebSocketRosPhysicalAdapterException;
import iot.drone.dt.adapter.physical.websocket.ros.rostopic.DigitalTwinRosTopic;
import iot.drone.dt.adapter.physical.websocket.ros.rostopic.RosTopicSubscribeFunction;
import iot.drone.dt.ros.RosCommands;
import iot.drone.dt.ros.px4_msgs.VehicleOdometry;
import iot.drone.dt.ros.px4_msgs.VehicleStatus;
import iot.drone.dt.utils.Vector3D;
import it.wldt.adapter.http.digital.adapter.HttpDigitalAdapter;
import it.wldt.adapter.http.digital.adapter.HttpDigitalAdapterConfiguration;
import it.wldt.adapter.mqtt.digital.MqttDigitalAdapter;
import it.wldt.adapter.mqtt.digital.MqttDigitalAdapterConfiguration;
import it.wldt.adapter.mqtt.digital.topic.MqttQosLevel;
import it.wldt.adapter.physical.event.PhysicalAssetPropertyWldtEvent;
import it.wldt.core.engine.DigitalTwin;
import it.wldt.core.engine.DigitalTwinEngine;
import it.wldt.core.event.WldtEvent;
import it.wldt.exception.*;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class DroneDigitalTwin {

    private static final Logger logger = LoggerFactory.getLogger(WebSocketRosPhysicalAdapterConfiguration.class);

    public static final String BASE_TOPIC = "/iot/user/";

    private final static Logger px4Logger = LoggerFactory.getLogger(DroneDigitalTwin.class);
    public static final String SIMULATED_DRONE_ID = "mavros";

    public static void main(String[] args) throws WldtConfigurationException, WldtRuntimeException,
            EventBusException, InterruptedException, WebSocketRosPhysicalAdapterException, NoSuchMethodException, MqttException, WldtWorkerException, WldtDigitalTwinStateException {

        try {

            DigitalTwinEngine digitalTwinEngine = new DigitalTwinEngine();

            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_1", 9090, 1, 3001, 9400,"ws-server-1", "pressure", "config-B", "zone-4", "forest"));

            digitalTwinEngine.startAll();

        } catch (Exception e) {
            e.printStackTrace();
        }

    }


    private static DigitalTwin createDigitalTwin(String droneId, Integer websocketServerPort, int targetSystem, Integer httpAdapterPort, Integer httpPrometheusServerPort,
                                                 String websocketServerId, String sensorType, String commType,
                                                 String zoneId, String zoneClass) {

        String SIMULATED_DRONE_ID = "/" + droneId;

        try {
            DroneDigitalTwinShadowingFunction defaultShadowingFunction = new DroneDigitalTwinShadowingFunction(droneId, droneId, websocketServerId);
            DigitalTwin digitalTwin = new DigitalTwin(
                    droneId + "px4-digital-twin-id",
                    defaultShadowingFunction
            );

            RosCommands rosCommand = new RosCommands(droneId, targetSystem);
            RosBridgeClient rosBridgeClient1 = new RosBridgeClient("192.168.56.106", websocketServerPort); // rosclient created
            WebSocketRosPhysicalAdapterConfiguration PhysicalAdapter1Config = new WebSocketRosPhysicalAdapterConfiguration(
                    droneId + "wesocket-adapter-config-1", rosBridgeClient1, rosCommand);

            // Add arm command to physical adapter config
            Method px4_arm = rosCommand.getClass().getDeclaredMethod("px4Arm", byte[].class,
                    RosBridge.class); // Arm
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("arm", "da.digital.action.event",
                    "byte", px4_arm); // Add arming action to physical adapter config

            // Add takeoff command to physical adapter config
            Method px4_take_off = rosCommand.getClass().getDeclaredMethod("px4Takeoff", byte[].class,
                    RosBridge.class);
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("takeoff", "da.digital.action.event",
                    "byte", px4_take_off);

            // Add offboard control to physical adapter config
            Method px4_offboard_control = rosCommand.getClass().getDeclaredMethod("px4Offboard", byte[].class,
                    RosBridge.class);
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("offboard_mode", "da.digital.action.event",
                    "byte", px4_offboard_control);

            // Add controller heartbeat to physical adapter config
            Method px4_pose_hbeat = rosCommand.getClass().getDeclaredMethod("px4TrajectorySetopoint", byte[].class, RosBridge.class); // Pose hbeat (need to send it continously to maintain contact with the drone
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("set_trajectory_setpoint", "da.digital.action.event",
                    "byte", px4_pose_hbeat);

            // Add land command to physical adapter config
            Method px4_land = rosCommand.getClass().getDeclaredMethod("px4Land", byte[].class,
                    RosBridge.class);
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("land", "da.digital.action.event",
                    "byte", px4_land);

            // Add disarm command to physical adapter config
            Method px4_disarm = rosCommand.getClass().getDeclaredMethod("px4Disarm", byte[].class, RosBridge.class); // Disarm
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("disarm", "da.digital.action.event",
                    "byte", px4_disarm);

            RosSubscription vehicleOdometryTopic = RosSubscription.builder(SIMULATED_DRONE_ID + "/fmu/out/vehicle_odometry",
                    VehicleOdometry.TYPE).queueLength(0).throttleRate(50).build();
            DigitalTwinRosTopic odometryTopic = new DigitalTwinRosTopic(vehicleOdometryTopic, getVehicleOdometry());

            Map<String, Integer> odometry = new HashMap<>();
            odometry.put("position", 0);
            odometry.put("velocity", 0);
            PhysicalAdapter1Config.addMultiplePhysicalAssetPropertyTopics(odometry, odometryTopic);

            RosSubscription vehicleStatusSubscription = RosSubscription.builder(SIMULATED_DRONE_ID + "/fmu/out/vehicle_status_v1",
                    VehicleStatus.TYPE).queueLength(0).throttleRate(50).build();
            DigitalTwinRosTopic vehicleStatusTopic = new DigitalTwinRosTopic(vehicleStatusSubscription, getVehicleStatus());
            PhysicalAdapter1Config.addPhysicalAssetPropertyTopic("status",
                    new VehicleStatus(), vehicleStatusTopic);

            PhysicalAdapter1Config.build();

            digitalTwin.addPhysicalAdapter(new WebSocketRosPhysicalAdapter(droneId + "test-ws-pa-1", PhysicalAdapter1Config));


            // HTTP DIGITAL ADAPTER
            HttpDigitalAdapterConfiguration http_digital_adapter_config = new HttpDigitalAdapterConfiguration("test-http-da",
                    "localhost", httpAdapterPort);
            http_digital_adapter_config.addActionFilter(droneId+"-arm");
            http_digital_adapter_config.addActionFilter(droneId+"-takeoff");
            //http_digital_adapter_config.addActionFilter("offboard_control");
            http_digital_adapter_config.addActionFilter(droneId+"-pose");
            http_digital_adapter_config.addActionFilter(droneId+"-land");
            http_digital_adapter_config.addActionFilter(droneId+"-disarm");
            http_digital_adapter_config.addActionFilter("waypoints");
            http_digital_adapter_config.addPropertyFilter("position");
            http_digital_adapter_config.addPropertyFilter("status");
            //http_digital_adapter_config.addActionFilter("stabilize");
            digitalTwin.addDigitalAdapter(new HttpDigitalAdapter(http_digital_adapter_config, digitalTwin));


            MqttDigitalAdapterConfiguration mqttDigitalAdapterConfiguration = MqttDigitalAdapterConfiguration.builder(
                            "localhost", 1883, droneId + "-mqtt-client")
                    .addActionTopic("swarm-takeoff", BASE_TOPIC + "swarm/action/takeoff", Function.identity())
                    .addActionTopic("swarm-land", BASE_TOPIC + "swarm/action/land", Function.identity())
                    .addActionTopic("swarm-arm", BASE_TOPIC + "swarm/action/arm", Function.identity())
                    .addActionTopic("takeoff", BASE_TOPIC + droneId + "/action/takeoff", Function.identity())
                    .addActionTopic("land", BASE_TOPIC + droneId + "/action/land", Function.identity())
                    .addActionTopic("offboard_control", BASE_TOPIC + droneId + "/action/offboard", Function.identity())
                    .addActionTopic("arm", BASE_TOPIC + droneId + "/action/arm", Function.identity())
                    .addActionTopic("pose", BASE_TOPIC + droneId + "/action/pose", Function.identity()).build();
            //.addActionTopic("waypoints", BASE_TOPIC + SIMULATED_DRONE_ID + "/command", Function.identity())
            //.addActionTopic("disarm", BASE_TOPIC + SIMULATED_DRONE_ID + "/command", Function.identity())
            //.addActionTopic("stabilize", BASE_TOPIC + SIMULATED_DRONE_ID + "/command", Function.identity())

            digitalTwin.addDigitalAdapter(new MqttDigitalAdapter(droneId + "mqtt-da", mqttDigitalAdapterConfiguration));

            return digitalTwin;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }


    private static RosTopicSubscribeFunction getVehicleOdometry() {
        return msgPayload -> {
            WldtEvent<?> positionEvent = null;
            WldtEvent<?> velocityEvent = null;
            VehicleOdometry vehicleOdometry =
                    VehicleOdometry.fromJsonObject(msgPayload);

            try {
                Vector3D position = vehicleOdometry.getPositionAsArray();

                Vector3D velocity = vehicleOdometry.getVelocityAsArray();

                positionEvent = new PhysicalAssetPropertyWldtEvent<JsonObject>(
                        "position",
                        position.getJsonObject()
                );
                velocityEvent = new PhysicalAssetPropertyWldtEvent<JsonObject>(
                        "velocity",
                        velocity.getJsonObject()
                );
            } catch (EventBusException e) {
                e.printStackTrace();
            }
            List<WldtEvent<?>> events = List.of(positionEvent, velocityEvent);
            return events;
        };
    }

    private static RosTopicSubscribeFunction getVehicleStatus() {
        return msgPayload -> {
            WldtEvent<?> statusEvent = null;

            VehicleStatus vehicleStatus =
                    VehicleStatus.fromJsonObject(msgPayload);

            String status = vehicleStatus.getStatusSummary();

            try {
                statusEvent = new PhysicalAssetPropertyWldtEvent<String>(
                        "status",
                        status
                );
            } catch (EventBusException e) {
                e.printStackTrace();
            }

            List<WldtEvent<?>> events = List.of(statusEvent);
            return events;
        };
    }

}
