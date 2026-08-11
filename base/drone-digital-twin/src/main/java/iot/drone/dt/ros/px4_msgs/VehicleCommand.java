package iot.drone.dt.ros.px4_msgs;

import io.github.twinklekhj.ros.type.RosMessage;
import io.vertx.core.json.JsonObject;
import lombok.Getter;
import lombok.ToString;

@ToString
@Getter
public class VehicleCommand extends RosMessage {
    public static final String TYPE = "px4_msgs/msg/VehicleCommand";

    // PX4 Commands codes
    public static final int VEHICLE_CMD_DO_SET_MODE = 176;
    public static final int VEHICLE_CMD_NAV_TAKEOFF = 22;
    public static final int VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
    public static final int VEHICLE_CMD_SET_NAV_STATE = 100001;

    // ==================== FIELDS ====================
    public static final String FIELD_TIMESTAMP = "timestamp";
    public static final String FIELD_PARAM1 = "param1";
    public static final String FIELD_PARAM2 = "param2";
    public static final String FIELD_PARAM3 = "param3";
    public static final String FIELD_PARAM4 = "param4";
    public static final String FIELD_PARAM5 = "param5";
    public static final String FIELD_PARAM6 = "param6";
    public static final String FIELD_PARAM7 = "param7";
    public static final String FIELD_COMMAND = "command";
    public static final String FIELD_TARGET_SYSTEM = "target_system";
    public static final String FIELD_TARGET_COMPONENT = "target_component";
    public static final String FIELD_SOURCE_SYSTEM = "source_system";
    public static final String FIELD_SOURCE_COMPONENT = "source_component";
    public static final String FIELD_CONFIRMATION = "confirmation";
    public static final String FIELD_FROM_EXTERNAL = "from_external";

    private long timestamp;
    private float param1;
    private float param2;
    private float param3;
    private float param4;
    private double param5;
    private double param6;
    private float param7;
    private int command;
    private int targetSystem;
    private int targetComponent;
    private int sourceSystem;
    private int sourceComponent;
    private int confirmation;
    private boolean fromExternal;

    public VehicleCommand() {
        this(0L, 0.0f, 0.0f, 0.0f, 0.0f, 0.0, 0.0, 0.0f, 0, 1, 1, 1, 1, 0, true);
    }

    public VehicleCommand(long timestamp, float param1, float param2, float param3, float param4,
                          double param5, double param6, float param7, int command, int targetSystem,
                          int targetComponent, int sourceSystem, int sourceComponent, int confirmation, boolean fromExternal) {
        this.timestamp = timestamp;
        this.param1 = param1;
        this.param2 = param2;
        this.param3 = param3;
        this.param4 = param4;
        this.param5 = param5;
        this.param6 = param6;
        this.param7 = param7;
        this.command = command;
        this.targetSystem = targetSystem;
        this.targetComponent = targetComponent;
        this.sourceSystem = sourceSystem;
        this.sourceComponent = sourceComponent;
        this.confirmation = confirmation;
        this.fromExternal = fromExternal;

        JsonObject obj = new JsonObject()
                .put(FIELD_TIMESTAMP, timestamp)
                .put(FIELD_PARAM1, param1)
                .put(FIELD_PARAM2, param2)
                .put(FIELD_PARAM3, param3)
                .put(FIELD_PARAM4, param4)
                .put(FIELD_PARAM5, param5)
                .put(FIELD_PARAM6, param6)
                .put(FIELD_PARAM7, param7)
                .put(FIELD_COMMAND, command)
                .put(FIELD_TARGET_SYSTEM, targetSystem)
                .put(FIELD_TARGET_COMPONENT, targetComponent)
                .put(FIELD_SOURCE_SYSTEM, sourceSystem)
                .put(FIELD_SOURCE_COMPONENT, sourceComponent)
                .put(FIELD_CONFIRMATION, confirmation)
                .put(FIELD_FROM_EXTERNAL, fromExternal);

        super.setJsonObject(obj);
        super.setType(TYPE);
    }

    public static VehicleCommand fromJsonObject(JsonObject jsonObject) {
        return new VehicleCommand(
                jsonObject.getLong(FIELD_TIMESTAMP, 0L),
                jsonObject.getFloat(FIELD_PARAM1, 0.0f),
                jsonObject.getFloat(FIELD_PARAM2, 0.0f),
                jsonObject.getFloat(FIELD_PARAM3, 0.0f),
                jsonObject.getFloat(FIELD_PARAM4, 0.0f),
                jsonObject.getDouble(FIELD_PARAM5, 0.0),
                jsonObject.getDouble(FIELD_PARAM6, 0.0),
                jsonObject.getFloat(FIELD_PARAM7, 0.0f),
                jsonObject.getInteger(FIELD_COMMAND, 0),
                jsonObject.getInteger(FIELD_TARGET_SYSTEM, 0),
                jsonObject.getInteger(FIELD_TARGET_COMPONENT, 0),
                jsonObject.getInteger(FIELD_SOURCE_SYSTEM, 0),
                jsonObject.getInteger(FIELD_SOURCE_COMPONENT, 0),
                jsonObject.getInteger(FIELD_CONFIRMATION, 0),
                jsonObject.getBoolean(FIELD_FROM_EXTERNAL, false)
        );
    }

    @Override
    public VehicleCommand clone() {
        return new VehicleCommand(this.timestamp, this.param1, this.param2, this.param3, this.param4,
                this.param5, this.param6, this.param7, this.command, this.targetSystem,
                this.targetComponent, this.sourceSystem, this.sourceComponent,
                this.confirmation, this.fromExternal);
    }
}
