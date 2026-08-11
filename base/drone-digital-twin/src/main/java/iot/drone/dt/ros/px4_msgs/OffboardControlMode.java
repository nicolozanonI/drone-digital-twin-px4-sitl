package iot.drone.dt.ros.px4_msgs;

import io.github.twinklekhj.ros.type.RosMessage;
import io.vertx.core.json.JsonObject;
import lombok.Getter;
import lombok.ToString;

@ToString
@Getter
public class OffboardControlMode extends RosMessage {
    public static final String TYPE = "px4_msgs/msg/OffboardControlMode";

    // ==================== FIELDS ====================
    public static final String FIELD_TIMESTAMP = "timestamp";
    public static final String FIELD_POSITION = "position";
    public static final String FIELD_VELOCITY = "velocity";
    public static final String FIELD_ACCELERATION = "acceleration";
    public static final String FIELD_ATTITUDE = "attitude";
    public static final String FIELD_BODY_RATE = "body_rate";
    public static final String FIELD_THRUST_AND_TORQUE = "thrust_and_torque";
    public static final String FIELD_DIRECT_ACTUATOR = "direct_actuator";

    private long timestamp;
    private boolean position;
    private boolean velocity;
    private boolean acceleration;
    private boolean attitude;
    private boolean bodyRate;
    private boolean thrustAndTorque;
    private boolean directActuator;

    public OffboardControlMode() {
        this(0L, false, false, false, false, false, false, false);
    }

    public OffboardControlMode(long timestamp, boolean position, boolean velocity, boolean acceleration,
                               boolean attitude, boolean bodyRate, boolean thrustAndTorque, boolean directActuator) {
        this.timestamp = timestamp;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.attitude = attitude;
        this.bodyRate = bodyRate;
        this.thrustAndTorque = thrustAndTorque;
        this.directActuator = directActuator;

        JsonObject obj = new JsonObject()
                .put(FIELD_TIMESTAMP, timestamp)
                .put(FIELD_POSITION, position)
                .put(FIELD_VELOCITY, velocity)
                .put(FIELD_ACCELERATION, acceleration)
                .put(FIELD_ATTITUDE, attitude)
                .put(FIELD_BODY_RATE, bodyRate)
                .put(FIELD_THRUST_AND_TORQUE, thrustAndTorque)
                .put(FIELD_DIRECT_ACTUATOR, directActuator);

        super.setJsonObject(obj);
        super.setType(TYPE);
    }

    public static OffboardControlMode fromJsonString(String jsonString) {
        return OffboardControlMode.fromMessage(new RosMessage(jsonString, TYPE));
    }

    public static OffboardControlMode fromMessage(RosMessage m) {
        return OffboardControlMode.fromJsonObject(m.getJsonObject());
    }

    public static OffboardControlMode fromJsonObject(JsonObject jsonObject) {
        return new OffboardControlMode(
                jsonObject.getLong(FIELD_TIMESTAMP, 0L),
                jsonObject.getBoolean(FIELD_POSITION, false),
                jsonObject.getBoolean(FIELD_VELOCITY, false),
                jsonObject.getBoolean(FIELD_ACCELERATION, false),
                jsonObject.getBoolean(FIELD_ATTITUDE, false),
                jsonObject.getBoolean(FIELD_BODY_RATE, false),
                jsonObject.getBoolean(FIELD_THRUST_AND_TORQUE, false),
                jsonObject.getBoolean(FIELD_DIRECT_ACTUATOR, false)
        );
    }

    @Override
    public OffboardControlMode clone() {
        return new OffboardControlMode(this.timestamp, this.position, this.velocity, this.acceleration,
                this.attitude, this.bodyRate, this.thrustAndTorque, this.directActuator);
    }
}
