package iot.drone.dt;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.gson.Gson;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import iot.drone.dt.utils.Vector3D;
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

    private Boolean isOffboard = false;

    private Boolean set_offboard = false;

    Logger px4_shadowing_logger = LoggerFactory.getLogger(DroneDigitalTwinShadowingFunction.class);

    private Vector3D pose_cmd = new Vector3D(0.0F, 0.0F, -5.0F);

    private byte[] pose_cmd_bytes = pose_cmd.toString().getBytes();

    private Boolean isArmed = false;

    private Boolean isMoving = false;

    private String droneId = "";

    private String websocketServerId = "";



    long benchmarkRate = 10;

    public DroneDigitalTwinShadowingFunction(String id, String droneId, String wsId) {
        super(id);
        this.droneId = droneId;
        this.websocketServerId = wsId;

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
        try {
            DigitalTwinStateEvent digitalTwinStateEvent = new DigitalTwinStateEvent("state-change", "dt.event");
            this.digitalTwinStateManager.registerEvent(digitalTwinStateEvent);
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            this.digitalTwinStateManager.commitStateTransaction();
            observeDigitalActionEvents();
            notifyShadowingSync();
            this.onStart();

        } catch (WldtDigitalTwinStateException | EventBusException e) {
            throw new RuntimeException(e);
        }
        offboardControlMode();


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
                        this.digitalTwinStateManager.startStateTransaction();
                        this.digitalTwinStateManager.updateProperty(
                                new DigitalTwinStateProperty<>(
                                        physicalAssetPropertyWldtEvent.getPhysicalPropertyId(),
                                        physicalAssetPropertyWldtEvent.getBody()));
                        this.digitalTwinStateManager.notifyDigitalTwinStateEvent(new DigitalTwinStateEventNotification<>("state-change", physicalAssetPropertyWldtEvent.getBody(), System.currentTimeMillis()));
                        this.digitalTwinStateManager.commitStateTransaction();
                        break;
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

    }

    @Override
    protected void onPhysicalAssetEventNotification(PhysicalAssetEventWldtEvent<?> physicalAssetEventWldtEvent) {
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
                } catch (EventBusException e) {
                    throw new RuntimeException(e);
                }
                break;*/
            case "pose":
                Vector3D positionArray = parseMyArray((byte[]) digitalActionWldtEvent.getBody().toString().getBytes());
                assert positionArray != null;
                this.pose_cmd = positionArray;
                this.pose_cmd_bytes = (byte[]) pose_cmd.toString().getBytes();
                break;
            case "waypoints":
                try {
                    Vector3D[] positionArrays = parseWaypoints(digitalActionWldtEvent.getBody().toString().getBytes());
                    assert positionArrays != null;
                    Vector3D new_position = positionArrays[0];
                    assert new_position != null;
                    this.pose_cmd = new_position;
                    this.pose_cmd_bytes = (byte[]) pose_cmd.toString().getBytes();
                    Integer i = 1;
                    Integer j = 0;
                    while (i != positionArrays.length) {
                        Vector3D actualPosition = (Vector3D) this.digitalTwinStateManager.getDigitalTwinState().getProperty("position").get().getValue();
                        Thread.sleep(100);
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
                            Thread.sleep(100);
                        }
                        else {
                            Thread.sleep(100);
                        }
                    }
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
                } catch (EventBusException e) {
                    throw new RuntimeException(e);
                }
                break;

            case "traslation":
                try {
                    Vector3D traslationArray = parseMyArray((byte[]) digitalActionWldtEvent.getBody().toString().getBytes());
                    assert traslationArray != null;
                    Vector3D currentPosition = (Vector3D) this.digitalTwinStateManager.getDigitalTwinState().getProperty("position").get().getValue();
                    Vector3D sum = traslationArray.add(currentPosition);
                    this.pose_cmd = sum;
                    this.pose_cmd_bytes = (byte[]) pose_cmd.toString().getBytes();
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


    public static Vector3D parseMyArray(byte[] payload) {
        try {
            Gson gson = new Gson();
            return gson.fromJson(new String(payload), Vector3D.class);
        }catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static String buildJsonPositionArrayMessage(Vector3D positionArray){
        try {
            Gson gson = new Gson();
            return gson.toJson(positionArray);
        }catch(Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static Vector3D[] parseWaypoints(byte[] payload) {
        try {
            Gson gson = new Gson();
            Vector3D[] wayPoints = gson.fromJson(new String(payload), Vector3D[].class);
            return wayPoints;
        }catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

}



