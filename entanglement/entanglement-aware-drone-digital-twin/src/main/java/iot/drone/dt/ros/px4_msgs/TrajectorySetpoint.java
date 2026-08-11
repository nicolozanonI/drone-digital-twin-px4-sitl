package iot.drone.dt.ros.px4_msgs;

import io.github.twinklekhj.ros.type.RosMessage;
import io.vertx.core.json.JsonArray;
import io.vertx.core.json.JsonObject;
import lombok.Getter;
import lombok.ToString;

@ToString
@Getter
public class TrajectorySetpoint extends RosMessage {
    public static final String TYPE = "px4_msgs/msg/TrajectorySetpoint";

    // ==================== FIELDS ====================
    public static final String FIELD_TIMESTAMP = "timestamp";
    public static final String FIELD_POSITION = "position";
    public static final String FIELD_VELOCITY = "velocity";
    public static final String FIELD_ACCELERATION = "acceleration";
    public static final String FIELD_JERK = "jerk";
    public static final String FIELD_YAW = "yaw";
    public static final String FIELD_YAW_SPEED = "yawspeed";

    private long timestamp;
    private float[] position;     // [x, y, z] in metri (NED)
    private float[] velocity;     // [vx, vy, vz]
    private float[] acceleration;
    private float[] jerk;
    private float yaw;
    private float yawSpeed;

    public TrajectorySetpoint() {
        // Inizializziamo con NaN per evitare che il drone cerchi di andare a (0,0,0) involontariamente
        this(0L, createNanArray(3), createNanArray(3), createNanArray(3), createNanArray(3), 0.0f, 0.0f);
    }

    public TrajectorySetpoint(long timestamp, float[] position, float[] velocity, float[] acceleration,
                              float[] jerk, float yaw, float yawSpeed) {
        this.timestamp = timestamp;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
        this.yaw = yaw;
        this.yawSpeed = yawSpeed;

        JsonObject obj = new JsonObject()
                .put(FIELD_TIMESTAMP, timestamp)
                // NON passare position direttamente, usa l'helper
                .put(FIELD_POSITION, floatToJsonArray(position))
                .put(FIELD_VELOCITY, floatToJsonArray(velocity))
                .put(FIELD_ACCELERATION, floatToJsonArray(acceleration))
                .put(FIELD_JERK, floatToJsonArray(jerk))
                .put(FIELD_YAW, yaw)
                .put(FIELD_YAW_SPEED, yawSpeed);

        super.setJsonObject(obj);
        super.setType(TYPE);
    }

    public static TrajectorySetpoint fromJsonObject(JsonObject jsonObject) {
        return new TrajectorySetpoint(
                jsonObject.getLong(FIELD_TIMESTAMP, 0L),
                toFloatArray(jsonObject.getJsonArray(FIELD_POSITION), 3),
                toFloatArray(jsonObject.getJsonArray(FIELD_VELOCITY), 3),
                toFloatArray(jsonObject.getJsonArray(FIELD_ACCELERATION), 3),
                toFloatArray(jsonObject.getJsonArray(FIELD_JERK), 3),
                jsonObject.getFloat(FIELD_YAW, 0.0f),
                jsonObject.getFloat(FIELD_YAW_SPEED, 0.0f)
        );
    }

    private static JsonArray floatToJsonArray(float[] values) {
        JsonArray array = new JsonArray();
        if (values != null) {
            for (float f : values) {
                array.add(f);
            }
        }
        return array;
    }

    @Override
    public TrajectorySetpoint clone() {
        return new TrajectorySetpoint(this.timestamp, this.position.clone(), this.velocity.clone(),
                this.acceleration.clone(), this.jerk.clone(), this.yaw, this.yawSpeed);
    }

    // Utility to manage NaN values
    private static float[] createNanArray(int size) {
        float[] arr = new float[size];
        for (int i = 0; i < size; i++) arr[i] = Float.NaN;
        return arr;
    }

    private static float[] toFloatArray(JsonArray array, int size) {
        float[] result = new float[size];
        if (array != null) {
            for (int i = 0; i < Math.min(size, array.size()); i++) {
                result[i] = array.getFloat(i);
            }
        }
        return result;
    }
}
