package iot.drone.dt;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.gson.Gson;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import io.vertx.core.json.JsonObject;
import iot.drone.dt.modules.BenchmarkModule;
import iot.drone.dt.modules.CsvExporter;
import iot.drone.dt.modules.OdteManager;
import iot.drone.dt.ros.px4_msgs.CustomVehicleOdometry;
import iot.drone.dt.utils.Components3D;
import iot.drone.dt.utils.TimestampedNotification;
import it.wldt.adapter.digital.event.DigitalActionWldtEvent;
import it.wldt.adapter.physical.PhysicalAssetDescription;
import it.wldt.adapter.physical.PhysicalAssetProperty;
import it.wldt.adapter.physical.event.PhysicalAssetEventWldtEvent;
import it.wldt.adapter.physical.event.PhysicalAssetPropertyWldtEvent;
import it.wldt.adapter.physical.event.PhysicalAssetRelationshipInstanceCreatedWldtEvent;
import it.wldt.adapter.physical.event.PhysicalAssetRelationshipInstanceDeletedWldtEvent;
import it.wldt.core.model.DigitalTwinModel;
import it.wldt.core.state.DigitalTwinStateAction;
import it.wldt.core.state.DigitalTwinStateEvent;
import it.wldt.core.state.DigitalTwinStateEventNotification;
import it.wldt.core.state.DigitalTwinStateProperty;
import it.wldt.exception.*;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.time.*;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;

public class DroneDigitalTwinShadowingFunction extends DigitalTwinModel{

    private final BlockingQueue<TimestampedNotification> actionsNotifications = new LinkedBlockingQueue<>();

    public OdteManager odteManager;

    private Boolean isOffboard = false;

    private Boolean set_offboard = false;

    Logger px4_shadowing_logger = LoggerFactory.getLogger(DroneDigitalTwinShadowingFunction.class);

    private Components3D pose_cmd = new Components3D(0.0F, 0.0F, -5.0F);

    private byte[] pose_cmd_bytes = pose_cmd.toString().getBytes();

    private Boolean isArmed = false;

    private Boolean isMoving = false;

    private Integer httpPrometheusServerPort;

    private String droneId = "";

    private String websocketServerId = "";

    private final AtomicLong benchmarkTimer = new AtomicLong(0);

    private final ConcurrentHashMap<Long, String> benchmarkDataset = new ConcurrentHashMap<>();

    BenchmarkModule benchmarkModule = new BenchmarkModule();

    String broker = "tcp://localhost:1883";
    private final String sensorType;
    private final String commType;
    private final String zoneId;
    private final String zoneClass;
    String nodeTopic = "graph/command/node/create";
    String edgeTopic = "graph/command/edge/connect";
    String edgeUpdateTopic = "graph/command/edge/update";

    long benchmarkRate = 10;

    public DroneDigitalTwinShadowingFunction(String id, OdteManager odteManager, Integer httpPrometheusServerPort, String droneId, String wsId,
                                       String sensorType, String commType,
                                       String zoneId, String zoneClass) {
        super(id);
        this.odteManager = odteManager;
        this.httpPrometheusServerPort = httpPrometheusServerPort;
        this.droneId = droneId;
        this.websocketServerId = wsId;
        this.sensorType = sensorType;
        this.commType = commType;
        this.zoneId = zoneId;
        this.zoneClass = zoneClass;

    }


    @Override
    protected void onCreate() {

    }

    @Override
    protected void onStart() {

    }

    @Override
    protected void onStop() {

    }

    @Override
    protected void onDigitalTwinBound(Map<String, PhysicalAssetDescription> map) {
        px4_shadowing_logger.debug("Shadowing - onDtBound");

        this.digitalTwinStateManager.startStateTransaction();

        map.values().forEach(pad -> {
            pad.getProperties().forEach(property -> {
                try {
                    this.digitalTwinStateManager.createProperty(new DigitalTwinStateProperty<>(property.getKey(),
                            property.getInitialValue()));
                    this.observePhysicalAssetProperty(property);

                } catch (Exception e) {
                    e.printStackTrace();
                }
            });
            pad.getEvents().forEach(event -> {
                try {

                    DigitalTwinStateEvent dtStateEvent = new DigitalTwinStateEvent(event.getKey(), event.getType());
                    this.digitalTwinStateManager.registerEvent(dtStateEvent);
                    this.observePhysicalAssetEvent(event);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            });

            pad.getActions().forEach(action -> {
                try {
                    DigitalTwinStateAction dtStateAction = new DigitalTwinStateAction(action.getKey(), action.getType(),
                            action.getContentType());
                    this.digitalTwinStateManager.enableAction(dtStateAction);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            });
        });

        try {
            PhysicalAssetProperty<Object> physicalAssetProperty = new PhysicalAssetProperty<>(droneId+"-entanglement", Objects.requireNonNull(buildJsonOdteMap(new HashMap<String, Double>() {{
                putAll(Map.of(
                        "odte_p2d", 1.0,
                        "reliability", 1.0,
                        "timeliness", 1.0,
                        "packets_validity", 1.0,
                        "out_of_sync_packets", 1.0
                ));
            }})));
            this.digitalTwinStateManager.createProperty(new DigitalTwinStateProperty<>(physicalAssetProperty.getKey(),
                    physicalAssetProperty.getInitialValue()));
            this.observePhysicalAssetProperty(physicalAssetProperty);
        } catch (WldtDigitalTwinStateException | EventBusException | KernelException e) {
            throw new RuntimeException(e);
        }

        /*try {
            PhysicalAssetProperty<Components3D> physicalAssetProperty = new PhysicalAssetProperty<>("position", new Components3D(0.0,0.0,0.0));
            this.digitalTwinStateManager.createProperty(new DigitalTwinStateProperty<>(physicalAssetProperty.getKey(),
                    physicalAssetProperty.getInitialValue()));
            this.observePhysicalAssetProperty(physicalAssetProperty);
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            PhysicalAssetProperty<Components3D> physicalAssetProperty = new PhysicalAssetProperty<>("velocity", new Components3D(0.0,0.0,0.0));
            this.digitalTwinStateManager.createProperty(new DigitalTwinStateProperty<>(physicalAssetProperty.getKey(),
                    physicalAssetProperty.getInitialValue()));
            this.observePhysicalAssetProperty(physicalAssetProperty);
        } catch (Exception e) {
            e.printStackTrace();
        }*/


        try {
            DigitalTwinStateAction dtStateAction = new DigitalTwinStateAction("offboard_control", "dt.action",
                    "None");
            this.digitalTwinStateManager.enableAction(dtStateAction);
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            DigitalTwinStateAction dtStateAction = new DigitalTwinStateAction("pose", "dt.action",
                    "3D_vector");
            this.digitalTwinStateManager.enableAction(dtStateAction);
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            DigitalTwinStateAction dtStateAction = new DigitalTwinStateAction("waypoints", "dt.action",
                    "3D_vector");
            this.digitalTwinStateManager.enableAction(dtStateAction);
        } catch (Exception e) {
            e.printStackTrace();
        }
        /*try {
            DigitalTwinStateAction positionRobustness = new DigitalTwinStateAction("robustness", "dt.action",
                    "3D_vector");
            this.digitalTwinStateManager.enableAction(positionRobustness);
        } catch (Exception e) {
            e.printStackTrace();
        }*/
        try {
            DigitalTwinStateEvent digitalTwinStateEvent = new DigitalTwinStateEvent("state-change", "dt.event");
            this.digitalTwinStateManager.registerEvent(digitalTwinStateEvent);
        } catch (Exception e) {
            e.printStackTrace();
        }
        /*try {
            DigitalTwinStateEvent digitalTwinStateEvent = new DigitalTwinStateEvent("take_off_action_notification", "dt.event");
            this.digitalTwinStateManager.registerEvent(digitalTwinStateEvent);
        } catch (Exception e) {
            e.printStackTrace();
        }*/


        try {
            this.digitalTwinStateManager.commitStateTransaction();
            observeDigitalActionEvents();
            notifyShadowingSync();
            this.onStart();

        } catch (WldtDigitalTwinStateException | EventBusException e) {
            throw new RuntimeException(e);
        }
        //this.digitalTwinStartTimestamp = System.currentTimeMillis() / 1000;

        //expectedRate.set(0.5);

        HttpServer server = null;
        try {
            server = HttpServer.create(new InetSocketAddress(httpPrometheusServerPort), 0);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        // Aggiungi un contesto per l'endpoint /prometheus
        server.createContext("/prometheus", new HttpHandler() {
            @Override
            public void handle(HttpExchange exchange) throws IOException {
                // Imposta l'intestazione Content-Type per Prometheus
                exchange.getResponseHeaders().set("Content-Type", "text/plain; version=0.0.4");
                // Ottieni le metriche in formato Prometheus
                String response = odteManager.prometheusRegistry.scrape();
                // Invia la risposta
                exchange.sendResponseHeaders(200, response.getBytes().length);
                try (OutputStream os = exchange.getResponseBody()) {
                    os.write(response.getBytes());
                }
            }
        });

        // Avvia il server in un thread separato
        new Thread(server::start).start();

        // Qui ci va funzione per verificare se velocità è maggiore di 0
        //startSendingOdteMetrics();
        offboardControlMode();

        try {
            MqttClient client = new MqttClient(broker, MqttClient.generateClientId());
            client.connect();

            // Drone
            publish(client, nodeTopic, String.format(
                    "{\"label\":\"Drone\",\"id\":\"%s\"}", droneId));

            publish(client, nodeTopic, String.format(
                    "{\"label\":\"Digital_Twin\",\"id\":\"%s\"}", "dt-"+droneId));

            // Websocket Server
            publish(client, nodeTopic, String.format(
                    "{\"label\":\"WebsocketServer\",\"id\":\"%s\",\"properties\":{\"type\":\"websocket\"}}", websocketServerId));

            // Sensore
            publish(client, nodeTopic, String.format(
                    "{\"label\":\"Payload\",\"id\":\"%s\"}",
                    sensorType
            ));

            // Communication Tech
            publish(client, nodeTopic, String.format(
                    "{\"label\":\"Configuration\",\"id\":\"%s\"}",
                    commType
            ));

            // Zone
            publish(client, nodeTopic, String.format(
                    "{\"label\":\"Operational_Zone\",\"id\":\"%s\",\"properties\":{\"class\":\"%s\"}}", zoneId, zoneClass));

            // Drone -> Server
            publish(client, edgeTopic, String.format(
                    "{\"from\":{\"label\":\"Drone\",\"id\":\"%s\"},\"to\":{\"label\":\"WebsocketServer\",\"id\":\"%s\"},\"relationship\":\"CONNECTED_TO\"}",
                    droneId, websocketServerId));

            // Drone -> Sensor
            publish(client, edgeTopic, String.format(
                    "{\"from\":{\"label\":\"Drone\",\"id\":\"%s\"},\"to\":{\"label\":\"Payload\",\"id\":\"%s\"},\"relationship\":\"EQUIPPED_WITH\"}",
                    droneId, sensorType
            ));

            // Drone -> Technology
            publish(client, edgeTopic, String.format(
                    "{\"from\":{\"label\":\"Drone\",\"id\":\"%s\"},\"to\":{\"label\":\"Configuration\",\"id\":\"%s\"},\"relationship\":\"CONFIGURED_WITH\"}",
                    droneId, commType
            ));
            // Drone -> Zone
            publish(client, edgeTopic, String.format(
                    "{\"from\":{\"label\":\"Drone\",\"id\":\"%s\"},\"to\":{\"label\":\"Operational_Zone\",\"id\":\"%s\"},\"relationship\":\"LOCATED_IN\"}",
                    droneId, zoneId));
            publish(client, edgeTopic, String.format(
                    """
                    {
                      "from": { "label": "Drone", "id": "%s" },
                      "to":   { "label": "Digital_Twin",   "id": "%s" },
                      "relationship": "REPLICATED_BY",
                      "attributes": {
                        "reliability": 1.0,
                        "timeliness": 1.0,
                        "packet validity": 1.0,
                        "packet order": 1.0,
                        "odte": 1.0
                      }
                    }
                    """,
                    droneId,
                    "dt-"+droneId
            ));

            //startBenchmark(10, "22:04:00", 1800); // Here you can configure benchmark start

            System.out.println("Graph updated: Drone and Server created and connected.");

            startSendingOdteMetrics(client, edgeUpdateTopic);

        } catch (Exception e) {
            e.printStackTrace();
        }


    }

    @Override
    protected void onDigitalTwinUnBound(Map<String, PhysicalAssetDescription> map, String s) {

    }

    @Override
    protected void onPhysicalAdapterBidingUpdate(String s, PhysicalAssetDescription physicalAssetDescription) {

    }

    @Override
    protected void onPhysicalAssetPropertyVariation(PhysicalAssetPropertyWldtEvent<?> physicalAssetPropertyWldtEvent) {
        px4_shadowing_logger.info("Shadowing - onPAPropertyVariation - property event: {} ", physicalAssetPropertyWldtEvent);
        //Update Digital Twin Status
        if(!Objects.isNull(physicalAssetPropertyWldtEvent)) {
            try {
                String propertyId = physicalAssetPropertyWldtEvent.getPhysicalPropertyId();
                switch (propertyId) {
                    case "status":
                        //this.odteManager.incrementUpdateCounter();
                        /*if (!Objects.equals(this.digitalTwinStateManager.getDigitalTwinState().getProperty("status").get().getValue(), physicalAssetPropertyWldtEvent.getBody())) {
                            this.onPhysicalAssetEventNotification(new PhysicalAssetEventWldtEvent<>("state-change", physicalAssetPropertyWldtEvent.getBody()));
                        }*/
                        this.digitalTwinStateManager.startStateTransaction();
                        this.digitalTwinStateManager.updateProperty(
                                new DigitalTwinStateProperty<>(
                                        physicalAssetPropertyWldtEvent.getPhysicalPropertyId(),
                                        physicalAssetPropertyWldtEvent.getBody()));
                        this.digitalTwinStateManager.commitStateTransaction();
                        break;
                    /*case "odometry":
                        this.digitalTwinStateManager.startStateTransaction();
                        CustomVehicleOdometry odometry = CustomVehicleOdometry.fromJsonObject((JsonObject) physicalAssetPropertyWldtEvent.getBody());
                        Components3D position = odometry.getPositionAsArray();
                        double posX = position.getX();
                        double posY = position.getY();
                        double posZ = position.getZ();

                        Components3D velocity = odometry.getVelocityAsArray();
                        double velX = velocity.getX();
                        double velY = velocity.getY();
                        double velZ = velocity.getZ();
                        this.digitalTwinStateManager.updateProperty(
                                new DigitalTwinStateProperty<>(
                                        "position",
                                        position));
                        this.digitalTwinStateManager.updateProperty(
                                new DigitalTwinStateProperty<>(
                                        "velocity",
                                        velocity));
                        this.digitalTwinStateManager.commitStateTransaction();

                        // Heartbeat
                        this.odteManager.incrementHeartbeatsCounter();
                        break;*/
                    default:
                        this.digitalTwinStateManager.startStateTransaction();
                        this.digitalTwinStateManager.updateProperty(
                                new DigitalTwinStateProperty<>(
                                        physicalAssetPropertyWldtEvent.getPhysicalPropertyId(),
                                        physicalAssetPropertyWldtEvent.getBody()));
                        this.digitalTwinStateManager.commitStateTransaction();
                        break;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            px4_shadowing_logger.info("Wldt event null");
        }

        /*try {
            if (Objects.equals(physicalAssetPropertyWldtEvent.getPhysicalPropertyId(), "state")) {
                if (!Objects.equals(this.digitalTwinStateManager.getDigitalTwinState().getProperty("state").get().getValue(), physicalAssetPropertyWldtEvent.getBody()))
                {
                    this.onPhysicalAssetEventNotification(new PhysicalAssetEventWldtEvent<>("state-change", physicalAssetPropertyWldtEvent.getBody()));
                }
                this.updatesCounter.increment(1.0);
            }
            this.digitalTwinStateManager.startStateTransaction();
            if (Objects.equals(physicalAssetPropertyWldtEvent.getPhysicalPropertyId(), "position")) {
                TimestapedPayload timestapedPayload = (TimestapedPayload) physicalAssetPropertyWldtEvent.getBody();
                this.digitalTwinStateManager.updateProperty(
                        new DigitalTwinStateProperty<>(
                                physicalAssetPropertyWldtEvent.getPhysicalPropertyId(),
                                physicalAssetPropertyWldtEvent.getBody()));
                this.digitalTwinStateManager.commitStateTransaction();
                // Timeliness
                //timelinessLinkedList.add(new Timeliness(timestapedPayload.getTimestampPhysicalAdapter(), physicalAssetPropertyWldtEvent.getCreationTimestamp(),System.currentTimeMillis() / 1000));
                this.hearthbeatsCounter.increment(1.0);
            }
            if (Objects.equals(physicalAssetPropertyWldtEvent.getPhysicalPropertyId(), "position")){
                //this.poseUpdatesCounter.inc(1L);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }*/
    }

    @Override
    protected void onPhysicalAssetEventNotification(PhysicalAssetEventWldtEvent<?> physicalAssetEventWldtEvent) {

        //px4_shadowing_logger.info("Shadowing - onPhysicalAssetEventNotification - received Event:{}", physicalAssetEventWldtEvent);
        switch (physicalAssetEventWldtEvent.getPhysicalEventKey()) {
            case "action_notification":
                //Long observationTimestamp = physicalAssetEventWldtEvent.getCreationTimestamp();
                TimestampedNotification timestampedNotification = (TimestampedNotification) physicalAssetEventWldtEvent.getBody();
                actionsNotifications.add(timestampedNotification);
                break;
            default:
                try {
                    this.digitalTwinStateManager.startStateTransaction();
                    this.digitalTwinStateManager.notifyDigitalTwinStateEvent(new DigitalTwinStateEventNotification<>(
                            physicalAssetEventWldtEvent.getPhysicalEventKey(),
                            (String) physicalAssetEventWldtEvent.getBody(),
                            physicalAssetEventWldtEvent.getCreationTimestamp()));
                    this.digitalTwinStateManager.commitStateTransaction();
                    /*this.digitalTwinStateManager.startStateTransaction();
                    this.digitalTwinStateManager.updateProperty(new DigitalTwinStateProperty<>(
                            physicalAssetEventWldtEvent.getPhysicalEventKey(),
                            physicalAssetEventWldtEvent.getBody().toString()));
                    this.digitalTwinStateManager.commitStateTransaction();*/
                } catch (WldtDigitalTwinStateEventNotificationException e) {
                    e.printStackTrace();
                } catch (WldtDigitalTwinStateException e) {
                    throw new RuntimeException(e);
                }
        }
    }

    @Override
    protected void onPhysicalAssetRelationshipEstablished(PhysicalAssetRelationshipInstanceCreatedWldtEvent<?> physicalAssetRelationshipInstanceCreatedWldtEvent) {

    }

    @Override
    protected void onPhysicalAssetRelationshipDeleted(PhysicalAssetRelationshipInstanceDeletedWldtEvent<?> physicalAssetRelationshipInstanceDeletedWldtEvent) {

    }

    @Override
    protected void onDigitalActionEvent(DigitalActionWldtEvent<?> digitalActionWldtEvent) {
        //logger.info("Shadowing - onDigitalActionEvent - received:{}", digitalActionWldtEvent);
        switch (digitalActionWldtEvent.getActionKey()) {
            /*case "offboard_control":
                controllerHeartbeat();
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                try {
                    publishPhysicalAssetActionWldtEvent("offboard_mode", new Components3D(10.0, 10.0, 10.0));
                    //verifyActionExecution("OFFBOARD", System.currentTimeMillis());
                } catch (EventBusException e) {
                    throw new RuntimeException(e);
                }
                break;*/
            case "pose":
                Components3D positionArray = parseMyArray((byte[]) digitalActionWldtEvent.getBody().toString().getBytes());
                assert positionArray != null;
                this.pose_cmd = positionArray;
                this.pose_cmd_bytes = (byte[]) pose_cmd.toString().getBytes();
                //verifyDroneMovement(true, System.currentTimeMillis());
                break;
            case "waypoints":
                try {
                    Components3D[] positionArrays = parseWaypoints(digitalActionWldtEvent.getBody().toString().getBytes());
                    //px4_shadowing_logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! {}", buildJsonPositionArrayMessage(positionArrays[0]));
                    //publishPhysicalAssetActionWldtEvent("pose", buildJsonPositionArrayMessage(positionArrays[0]));
                    assert positionArrays != null;
                    Components3D new_position = positionArrays[0];
                    assert new_position != null;
                    this.pose_cmd = new_position;
                    this.pose_cmd_bytes = (byte[]) pose_cmd.toString().getBytes();
                    Integer i = 1;
                    Integer j = 0;
                    //Timer.Sample sample = Timer.start();
                    while (i != positionArrays.length) {
                        Components3D actualPosition = (Components3D) this.digitalTwinStateManager.getDigitalTwinState().getProperty("position").get().getValue();
                        //Components3D actualPosition = position.getGenericData();
                        ;
                        Thread.sleep(100); // prima era 5000
                        if ((actualPosition.getX() - positionArrays[j].getX() < 0.75) &&
                                (actualPosition.getX() - positionArrays[j].getX() > -0.75) &&
                                (actualPosition.getY() - positionArrays[j].getY() < 0.75) &&
                                (actualPosition.getY() - positionArrays[j].getY() > -0.75) &&
                                (actualPosition.getZ() - positionArrays[j].getZ() < 0.75) &&
                                (actualPosition.getZ() - positionArrays[j].getZ() > -0.75)){ // and position is correct
                            // publish next command
                            j = j + 1;
                            new_position = positionArrays[j];
                            assert new_position != null;
                            this.pose_cmd = new_position;
                            this.pose_cmd_bytes = (byte[]) pose_cmd.toString().getBytes();
                            i = i + 1;
                            Thread.sleep(100); // prima era 3000
                        }
                        else {
                            Thread.sleep(100); // prima era 1000
                        }
                    }
                    //sample.stop(this.odteManager.waypointsTimer);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                break;
            case "swarm-takeoff":
            case "takeoff":
                try {
                    isMoving = false;
                    trajectorySetpointCommand();
                    publishPhysicalAssetActionWldtEvent("takeoff", digitalActionWldtEvent.getBody());
                    //verifyActionExecution("AUTO.TAKEOFF", System.currentTimeMillis());
                } catch (EventBusException e) {
                    throw new RuntimeException(e);
                }

                break;
            case "swarm-land":
            case "land":
                try {

                    publishPhysicalAssetActionWldtEvent("land", digitalActionWldtEvent.getBody());
                    isMoving = false;
                    //verifyActionExecution("AUTO.LAND", System.currentTimeMillis());
                } catch (EventBusException e) {
                    throw new RuntimeException(e);
                }

                break;
            case "swarm-arm":
            case "arm":
                try {

                    publishPhysicalAssetActionWldtEvent("arm", digitalActionWldtEvent.getBody());
                    //verifyDroneArming(true, System.currentTimeMillis());
                } catch (EventBusException e) {
                    throw new RuntimeException(e);
                }
                break;

            case "traslation":
                try {
                    Components3D traslationArray = parseMyArray((byte[]) digitalActionWldtEvent.getBody().toString().getBytes());
                    assert traslationArray != null;
                    Components3D currentPosition = (Components3D) this.digitalTwinStateManager.getDigitalTwinState().getProperty("position").get().getValue();
                    Components3D sum = traslationArray.add(currentPosition);
                    this.pose_cmd = sum;
                    this.pose_cmd_bytes = (byte[]) pose_cmd.toString().getBytes();
                    //verifyDroneArming(true, System.currentTimeMillis());
                } catch (WldtDigitalTwinStatePropertyException e) {
                    throw new RuntimeException(e);
                }
                break;

            default:
                try {

                    publishPhysicalAssetActionWldtEvent(digitalActionWldtEvent.getActionKey(), digitalActionWldtEvent.getBody());
                } catch (EventBusException e) {
                    throw new RuntimeException(e);
                }
                break;
        }
    }

    protected void offboardControlMode(){
        new Thread(() -> {
            try {
                this.isOffboard = true;
                while(this.isOffboard){
                    publishPhysicalAssetActionWldtEvent("offboard_mode", "");
                    Thread.sleep(50);
                }
            } catch (EventBusException e) {
                e.printStackTrace();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }

    protected void trajectorySetpointCommand(){
        new Thread(() -> {
            try {
                this.isMoving = true;
                while(this.isMoving){
                    publishPhysicalAssetActionWldtEvent("set_trajectory_setpoint", buildJsonPositionArrayMessage(pose_cmd));
                    Thread.sleep(50);
                }
            } catch (EventBusException e) {
                e.printStackTrace();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }


    public static Components3D parseMyArray(byte[] payload) {
        try {
            Gson gson = new Gson();
            return gson.fromJson(new String(payload), Components3D.class);
        }catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static String buildJsonPositionArrayMessage(Components3D positionArray){
        try {
            Gson gson = new Gson();
            return gson.toJson(positionArray);
        }catch(Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    private void startBenchmark(long benchmarkRate, String startTime, long durationSeconds)
            throws WldtDigitalTwinStatePropertyException {

        if (benchmarkRate <= 0) {
            throw new IllegalArgumentException("benchmarkRate deve essere maggiore di 0");
        }

        if (durationSeconds <= 0) {
            throw new IllegalArgumentException("durationSeconds deve essere maggiore di 0");
        }

        DateTimeFormatter fileNameFormatter =
                DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");

        String dateTimeForFileName =
                LocalDateTime.now().format(fileNameFormatter);

        final String csvFileName =
                droneId + "_risultati_benchmark_" + dateTimeForFileName + ".csv";

        LocalTime requestedStartTime = LocalTime.parse(startTime);

        ZoneId zoneId = ZoneId.systemDefault();

        ZonedDateTime now = ZonedDateTime.now(zoneId);
        ZonedDateTime startDateTime = now.with(requestedStartTime);

        /*
         * Se l'orario richiesto è già passato oggi,
         * il benchmark partirà domani a quell'orario.
         */
        if (!startDateTime.isAfter(now)) {
            startDateTime = startDateTime.plusDays(1);
        }

        long delayMs = Duration.between(now, startDateTime).toMillis();

        System.out.println("Benchmark programmato per: " + startDateTime);
        System.out.println("Durata benchmark: " + durationSeconds + " secondi");
        System.out.println("File CSV: " + csvFileName);

        benchmarkModule.getScheduler().schedule(() -> {

            final CsvExporter csvLogger;

            try {
                csvLogger = new CsvExporter(csvFileName);
            } catch (IOException e) {
                throw new RuntimeException("Errore durante la creazione del file CSV del benchmark", e);
            }

            System.out.println("Benchmark partito: " + ZonedDateTime.now(zoneId));

            ScheduledFuture<?> benchmarkTask = benchmarkModule.getScheduler().scheduleAtFixedRate(() -> {
                try {
                    /*
                     * Timestamp Unix in microsecondi.
                     * Esempio: 1782551403535386
                     */
                    Instant instant = Instant.now();

                    long timestampUs =
                            instant.getEpochSecond() * 1_000_000L +
                                    instant.getNano() / 1_000L;

                    Components3D currentPosition = (Components3D) this.digitalTwinStateManager
                            .getDigitalTwinState()
                            .getProperty("position")
                            .get()
                            .getValue();

                    Components3D currentVelocity = (Components3D) this.digitalTwinStateManager
                            .getDigitalTwinState()
                            .getProperty("velocity")
                            .get()
                            .getValue();

                    double posX = currentPosition.getX();
                    double posY = currentPosition.getY();
                    double posZ = currentPosition.getZ();

                    double velX = currentVelocity.getX();
                    double velY = currentVelocity.getY();
                    double velZ = currentVelocity.getZ();


                    HashMap<String, Double> odteMetricsValues = new HashMap<>();
                    odteMetricsValues.put("odte", this.odteManager.physicalToDigitalOdte.value());
                    odteMetricsValues.put("reliability", this.odteManager.reliabilityPhysicalToDigital.ratioGauge().value());
                    odteMetricsValues.put("timeliness", this.odteManager.physicalToDigitalTimeliness.ratioGauge().value());
                    odteMetricsValues.put("packet validity", this.odteManager.packetValidityMetric.qualityRatioGauge().value());
                    odteMetricsValues.put("packet order", this.odteManager.packetOutOfSeqMetric.qualityRatioGauge().value());

                    csvLogger.log(
                            timestampUs,
                            droneId,
                            posX,
                            posY,
                            posZ,
                            velX,
                            velY,
                            velZ,
                            this.odteManager.physicalToDigitalOdte.value(),
                            this.odteManager.physicalToDigitalTimeliness.ratioGauge().value(),
                            this.odteManager.reliabilityPhysicalToDigital.ratioGauge().value(),
                            this.odteManager.packetValidityMetric.qualityRatioGauge().value(),
                            this.odteManager.packetOutOfSeqMetric.qualityRatioGauge().value()
                    );

                } catch (Exception e) {
                    e.printStackTrace();
                }

            }, 0, benchmarkRate, TimeUnit.MILLISECONDS);

            benchmarkModule.getScheduler().schedule(() -> {
                try {
                    System.out.println("Benchmark terminato: " + ZonedDateTime.now(zoneId));
                    System.out.println("Arresto task benchmark e chiusura CSV...");

                    benchmarkTask.cancel(false);
                    csvLogger.close();

                    benchmarkModule.stop();

                    System.out.println("Benchmark salvato in: " + csvFileName);

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }, durationSeconds, TimeUnit.SECONDS);

        }, delayMs, TimeUnit.MILLISECONDS);
    }

    public static String buildJsonOdteMap(HashMap<String, Double> odteMap){
        try {
            Gson gson = new Gson();
            return gson.toJson(odteMap);
        }catch(Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static Components3D[] parseWaypoints(byte[] payload) {
        try {
            Gson gson = new Gson();
            Components3D[] wayPoints = gson.fromJson(new String(payload), Components3D[].class);
            return wayPoints;
        }catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void startSendingOdteMetrics(MqttClient client, String edgeUpdateTopic) {

        new Thread(() -> {
            while (true) {
                try {
                    HashMap<String, Double> odteMetricsValues = new HashMap<>();
                    odteMetricsValues.put("odte", this.odteManager.physicalToDigitalOdte.value());
                    odteMetricsValues.put("reliability", this.odteManager.reliabilityPhysicalToDigital.ratioGauge().value());
                    odteMetricsValues.put("timeliness", this.odteManager.physicalToDigitalTimeliness.ratioGauge().value());
                    odteMetricsValues.put("packet validity", this.odteManager.packetValidityMetric.qualityRatioGauge().value());
                    odteMetricsValues.put("packet order", this.odteManager.packetOutOfSeqMetric.qualityRatioGauge().value());
                    PhysicalAssetPropertyWldtEvent<Object> physicalAssetPropertyWldtEvent = new PhysicalAssetPropertyWldtEvent<>(this.droneId+"-entanglement", Objects.requireNonNull(buildJsonOdteMap(odteMetricsValues)));
                    onPhysicalAssetPropertyVariation(physicalAssetPropertyWldtEvent);
                    Map<String, Object> messageBody = new HashMap<>();
                    messageBody.put("from", Map.of("label", "Drone", "id", droneId));
                    messageBody.put("to", Map.of("label", "Digital_Twin", "id", "dt-" + droneId));
                    messageBody.put("relationship", "REPLICATED_BY");
                    messageBody.put("attributes", odteMetricsValues);
                    ObjectMapper objectMapper = new ObjectMapper();
                    String payload = objectMapper.writeValueAsString(messageBody);

                    publish(client, edgeTopic, payload);
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
        }).start();
    }


    private static void publish(MqttClient client, String topic, String payload) throws Exception {
        MqttMessage message = new MqttMessage(payload.getBytes(StandardCharsets.UTF_8));
        message.setQos(1);
        client.publish(topic, message);
    }

}



