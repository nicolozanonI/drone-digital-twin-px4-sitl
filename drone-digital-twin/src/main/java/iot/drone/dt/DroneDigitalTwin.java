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
import iot.drone.dt.modules.OdteManager;
import iot.drone.dt.modules.RosPacketErrorDetector;
import iot.drone.dt.ros.RosCommands;
import iot.drone.dt.ros.px4_msgs.CustomVehicleOdometry;
import iot.drone.dt.ros.px4_msgs.CustomVehicleStatus;
import iot.drone.dt.utils.Components3D;
import iot.drone.dt.utils.TimestampedNotification;
import it.wldt.adapter.http.digital.adapter.HttpDigitalAdapter;
import it.wldt.adapter.http.digital.adapter.HttpDigitalAdapterConfiguration;
import it.wldt.adapter.mqtt.digital.MqttDigitalAdapter;
import it.wldt.adapter.mqtt.digital.MqttDigitalAdapterConfiguration;
import it.wldt.adapter.mqtt.digital.topic.MqttQosLevel;
import it.wldt.adapter.physical.event.PhysicalAssetEventWldtEvent;
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

            // If you are using 1 drone, set targetSystem to 1
            //digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_4", 9091,1, 3004, 9403, "ws-server-1", "temperature", "config-B", "zone-1", "rural"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_1", 9090, 1, 3001, 9400,"ws-server-1", "pressure", "config-B", "zone-4", "forest"));

            /*digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_1", 9092, 2, 3001, 9400,
                    "ws-server-1", "pressure", "config-B", "zone-4", "forest"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_2", 9092, 3, 3002, 9401,
                    "ws-server-2", "temperature", "config-A", "zone-1", "rural"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_3", 9092,4, 3003, 9402, "ws-server-1", "temperature", "config-B", "zone-4", "forest"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_4", 9093,5, 3004, 9403, "ws-server-1", "temperature", "config-B", "zone-1", "rural"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_5", 9093, 6,3005, 9404, "ws-server-2", "camera", "config-A", "zone-7", "urban"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_6", 9093,7, 3006,9405, "ws-server-2", "humidity", "config-C", "zone-7", "urban"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_7", 9091,8, 3007, 9406, "ws-server-1", "camera", "config-A", "zone-1", "rural"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_8", 9091,9, 3008, 9407, "ws-server-2", "pressure", "config-C", "zone-1", "rural"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_9", 9091,10, 3009, 9408, "ws-server-2", "pressure", "config-B", "zone-7", "urban"));
            digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_10", 9093,11, 3010, 9410, "ws-server-1", "humidity", "config-C", "zone-4", "forest"));*/


            //digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_11", 9094,12, 3011, 9411, "ws-server-cluster-2", "temperature", "config-A", "zone-1", "rural"));
            //digitalTwinEngine.addDigitalTwin(createDigitalTwin("px4_12", 9094,13, 3012, 9412, "ws-server-cluster-1", "camera", "config-C", "zone-7", "urban"));

            digitalTwinEngine.startAll();

        } catch (Exception e) {
            e.printStackTrace();
        }

    }


    private static DigitalTwin createDigitalTwin(String droneId, Integer websocketServerPort, int targetSystem, Integer httpAdapterPort, Integer httpPrometheusServerPort,
                                                 String websocketServerId, String sensorType, String commType,
                                                 String zoneId, String zoneClass) {

        String SIMULATED_DRONE_ID = "/" + droneId;

        RosPacketErrorDetector rosPacketErrorDetector = new RosPacketErrorDetector();

        OdteManager odteManager = new OdteManager(droneId);

        try {
            DroneDigitalTwinShadowingFunction defaultShadowingFunction = new DroneDigitalTwinShadowingFunction(droneId, odteManager, httpPrometheusServerPort, droneId, websocketServerId, sensorType, commType,
                    zoneId, zoneClass);
            DigitalTwin digitalTwin = new DigitalTwin(
                    droneId + "px4-digital-twin-id",
                    defaultShadowingFunction
            );

            RosCommands rosCommand = new RosCommands(droneId, targetSystem);
            RosBridgeClient rosBridgeClient1 = new RosBridgeClient("192.168.56.106", websocketServerPort); // rosclient created
            WebSocketRosPhysicalAdapterConfiguration PhysicalAdapter1Config = new WebSocketRosPhysicalAdapterConfiguration(
                    droneId + "wesocket-adapter-config-1", rosBridgeClient1, rosCommand);

            // Command methods
            Method px4_arm = rosCommand.getClass().getDeclaredMethod("px4Arm", byte[].class,
                    RosBridge.class); // Arm
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("arm", "da.digital.action.event",
                    "byte", px4_arm); // Add arming action to physical adapter config

            Method px4_take_off = rosCommand.getClass().getDeclaredMethod("px4Takeoff", byte[].class,
                    RosBridge.class);
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("takeoff", "da.digital.action.event",
                    "byte", px4_take_off);

            Method px4_offboard_control = rosCommand.getClass().getDeclaredMethod("px4Offboard", byte[].class,
                    RosBridge.class);
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("offboard_mode", "da.digital.action.event",
                    "byte", px4_offboard_control);

            Method px4_pose_hbeat = rosCommand.getClass().getDeclaredMethod("px4TrajectorySetopoint", byte[].class, RosBridge.class); // Pose hbeat (need to send it continously to maintain contact with the drone
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("set_trajectory_setpoint", "da.digital.action.event",
                    "byte", px4_pose_hbeat);

            Method px4_land = rosCommand.getClass().getDeclaredMethod("px4Land", byte[].class,
                    RosBridge.class);
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("land", "da.digital.action.event",
                    "byte", px4_land);

            Method px4_disarm = rosCommand.getClass().getDeclaredMethod("px4Disarm", byte[].class, RosBridge.class); // Disarm
            PhysicalAdapter1Config.addPhysicalAssetActionMethod("disarm", "da.digital.action.event",
                    "byte", px4_disarm); // Add arming action to physical adapter config

            RosSubscription vehicleOdometryTopic = RosSubscription.builder(SIMULATED_DRONE_ID + "/forwarder/vehicle/odometry",
                    CustomVehicleOdometry.TYPE).queueLength(0).throttleRate(50).build();
            DigitalTwinRosTopic odometryTopic = new DigitalTwinRosTopic(vehicleOdometryTopic, getVehicleOdometry(odteManager, rosPacketErrorDetector));
            /*PhysicalAdapter1Config.addPhysicalAssetPropertyTopic("odometry",
                    new CustomVehicleOdometry(), odometryTopic);*/
            Map<String, Integer> odometry = new HashMap<>();
            odometry.put("position", 0);
            odometry.put("velocity", 0);

            PhysicalAdapter1Config.addMultiplePhysicalAssetPropertyTopics(odometry, odometryTopic);

            RosSubscription vehicleStatusSubscription = RosSubscription.builder(SIMULATED_DRONE_ID + "/forwarder/vehicle/status",
                    CustomVehicleStatus.TYPE).queueLength(0).throttleRate(50).build();
            DigitalTwinRosTopic vehicleStatusTopic = new DigitalTwinRosTopic(vehicleStatusSubscription, getVehicleStatus(odteManager, rosPacketErrorDetector));
            PhysicalAdapter1Config.addPhysicalAssetPropertyTopic("status",
                    new CustomVehicleStatus(), vehicleStatusTopic);


            // Action receive notification event
            /*RosSubscription actionNoptificationSub = RosSubscription.builder("/forwarder/action/notification",
                    TimestampedString.TYPE).build();
            DigitalTwinRosTopic actionNotificationTopic = new DigitalTwinRosTopic(actionNoptificationSub, getTimestampedNotification());
            PhysicalAdapter1Config.addPhysicalAssetEventTopic("action_notification", "Long", actionNotificationTopic);*/

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
            http_digital_adapter_config.addPropertyFilter(droneId+"-entanglement");
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
                    .addActionTopic("pose", BASE_TOPIC + droneId + "/action/pose", Function.identity())
                    .addPropertyTopic(droneId + "-entanglement", BASE_TOPIC + droneId + "/property/entanglement", MqttQosLevel.MQTT_QOS_1, Function.identity()).build();
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


    private static RosTopicSubscribeFunction getVehicleOdometry(
            OdteManager odteManager,
            RosPacketErrorDetector rosPacketErrorDetector
    ) {
        return msgPayload -> {
            WldtEvent<?> positionEvent = null;
            WldtEvent<?> velocityEvent = null;
            odteManager.incrementTotalPacketsCounter();

            String validationResult = rosPacketErrorDetector.messageValidationResult(
                    msgPayload,
                    "/forwarder/vehicle/odometry"
            );
            switch (validationResult) {
                case "VALID": {
                    px4Logger.info("VALID");
                    odteManager.physicalToDigitalTimeliness.tracker().recordEvent();
                    odteManager.incrementValidPacketsCounter();

                    CustomVehicleOdometry customVehicleOdometry =
                            CustomVehicleOdometry.fromJsonObject(msgPayload);

                    try {
                        Components3D position = customVehicleOdometry.getPositionAsArray();
                        //double posX = position.getX();
                        //double posY = position.getY();
                        //double posZ = position.getZ();

                        Components3D velocity = customVehicleOdometry.getVelocityAsArray();
                        //double velX = velocity.getX();
                        //double velY = velocity.getY();
                        //double velZ = velocity.getZ();
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

                    break;
                }
                case "DELAYED_TIMESTAMP": {
                    px4Logger.info("DELAYED TIMESTAMP");
                    odteManager.incrementValidPacketsCounter();
                    odteManager.incrementOutOfSyncPacketsCounter();
                    break;
                }

                case "OUT_OF_SEQUENCE": {
                    px4Logger.info("OUT OF SEQUENCE");
                    odteManager.incrementValidPacketsCounter();
                    odteManager.incrementOutOfSyncPacketsCounter();
                    break;
                }
                case "INVALID_TIMESTAMP": {
                    px4Logger.info("INVALID TIMESTAMP");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
                case "CORRUPTED": {
                    px4Logger.info("CORRUPTED");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
                case "INVALID_STATE_VALUE": {
                    px4Logger.info("INVALID STATE VALUE");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
                case "INVALID": {
                    px4Logger.info("INVALID");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
                default: {
                    px4Logger.warn("UNKNOWN VALIDATION RESULT: {}", validationResult);
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
            }
            List<WldtEvent<?>> events = List.of(positionEvent, velocityEvent);
            return events;
        };
    }

    private static RosTopicSubscribeFunction getVehicleStatus(
            OdteManager odteManager,
            RosPacketErrorDetector rosPacketErrorDetector
    ) {
        return msgPayload -> {
            WldtEvent<?> statusEvent = null;

            odteManager.incrementTotalPacketsCounter();

            String validationResult = rosPacketErrorDetector.messageValidationResult(
                    msgPayload,
                    "/forwarder/vehicle/status"
            );

            switch (validationResult) {

                case "VALID": {
                    px4Logger.info("VALID STATUS");
                    //odteManager.physicalToDigitalTimeliness.tracker().recordEvent();
                    odteManager.incrementUpdateCounter();
                    odteManager.incrementValidPacketsCounter();

                    CustomVehicleStatus customVehicleStatus =
                            CustomVehicleStatus.fromJsonObject(msgPayload);

                    String status = customVehicleStatus.getStatusSummary();

                    try {
                        statusEvent = new PhysicalAssetPropertyWldtEvent<String>(
                                "status",
                                status
                        );
                    } catch (EventBusException e) {
                        e.printStackTrace();
                    }
                    break;
                }

                case "DELAYED_TIMESTAMP": {
                    px4Logger.info("DELAYED TIMESTAMP STATUS");
                    odteManager.incrementValidPacketsCounter();
                    odteManager.incrementOutOfSyncPacketsCounter();
                    break;
                }

                case "OUT_OF_SEQUENCE": {
                    px4Logger.info("OUT OF SEQUENCE STATUS");
                    odteManager.incrementValidPacketsCounter();
                    odteManager.incrementOutOfSyncPacketsCounter();
                    break;
                }

                case "INVALID_TIMESTAMP": {
                    px4Logger.info("INVALID TIMESTAMP STATUS");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }

                case "CORRUPTED": {
                    px4Logger.info("CORRUPTED STATUS");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }

                case "INVALID_STATE_VALUE": {
                    px4Logger.info("INVALID STATE VALUE STATUS");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
                case "INVALID": {
                    px4Logger.info("INVALID STATUS");
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
                default: {
                    px4Logger.warn("UNKNOWN STATUS VALIDATION RESULT: {}", validationResult);
                    odteManager.incrementInvalidPacketsCounter();
                    break;
                }
            }
            List<WldtEvent<?>> events = List.of(statusEvent);
            return events;
        };
    }

}
