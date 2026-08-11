package iot.drone.dt.ros.px4_msgs;

import io.github.twinklekhj.ros.type.RosMessage;
import io.vertx.core.json.JsonArray;
import io.vertx.core.json.JsonObject;
import lombok.Getter;
import lombok.ToString;

@ToString
@Getter
public class VehicleOdometry extends RosMessage {
    public static final String TYPE = "px4_msgs/msg/VehicleOdometry";

    // ==================== FIELD NAMES ====================
    public static final String FIELD_TIMESTAMP = "timestamp";
    public static final String FIELD_TIMESTAMP_SAMPLE = "timestamp_sample";
    public static final String FIELD_POSITION = "position";
    public static final String FIELD_Q = "q";
    public static final String FIELD_VELOCITY = "velocity";
    public static final String FIELD_ANGULAR_VELOCITY = "angular_velocity";
    public static final String FIELD_POSE_FRAME = "pose_frame";
    public static final String FIELD_VELOCITY_FRAME = "velocity_frame";

    private long timestamp;
    private long timestampSample;
    private float[] position;        // [x, y, z] in metri (NED)
    private float[] q;               // Quaternione [w, x, y, z]
    private float[] velocity;        // [vx, vy, vz] in m/s (NED)
    private float[] angularVelocity; // [rad/s]
    private int poseFrame;
    private int velocityFrame;

    public VehicleOdometry() {
        this(0L, 0L, new float[3], new float[4], new float[3], new float[3], 0, 0);
    }

    public VehicleOdometry(long timestamp, long timestampSample, float[] position, float[] q,
                           float[] velocity, float[] angularVelocity, int poseFrame, int velocityFrame) {
        this.timestamp = timestamp;
        this.timestampSample = timestampSample;
        this.position = position;
        this.q = q;
        this.velocity = velocity;
        this.angularVelocity = angularVelocity;
        this.poseFrame = poseFrame;
        this.velocityFrame = velocityFrame;

        JsonObject obj = new JsonObject()
                .put(FIELD_TIMESTAMP, timestamp)
                .put(FIELD_TIMESTAMP_SAMPLE, timestampSample)
                .put(FIELD_POSITION, new JsonArray().addAll(new JsonObject().put("list", position).getJsonArray("list"))) // Helper logico per array
                .put(FIELD_Q, new JsonArray().addAll(new JsonObject().put("list", q).getJsonArray("list")))
                .put(FIELD_VELOCITY, new JsonArray().addAll(new JsonObject().put("list", velocity).getJsonArray("list")))
                .put(FIELD_ANGULAR_VELOCITY, new JsonArray().addAll(new JsonObject().put("list", angularVelocity).getJsonArray("list")))
                .put(FIELD_POSE_FRAME, poseFrame)
                .put(FIELD_VELOCITY_FRAME, velocityFrame);

        super.setJsonObject(obj);
        super.setType(TYPE);
    }

    public static VehicleOdometry fromJsonString(String jsonString) {
        return VehicleOdometry.fromMessage(new RosMessage(jsonString, TYPE));
    }

    public static VehicleOdometry fromMessage(RosMessage m) {
        return VehicleOdometry.fromJsonObject(m.getJsonObject());
    }

    public static VehicleOdometry fromJsonObject(JsonObject jsonObject) {
        long timestamp = jsonObject.getLong(FIELD_TIMESTAMP, 0L);
        long timestampSample = jsonObject.getLong(FIELD_TIMESTAMP_SAMPLE, 0L);

        float[] position = toFloatArray(jsonObject.getJsonArray(FIELD_POSITION), 3);
        float[] q = toFloatArray(jsonObject.getJsonArray(FIELD_Q), 4);
        float[] velocity = toFloatArray(jsonObject.getJsonArray(FIELD_VELOCITY), 3);
        float[] angularVelocity = toFloatArray(jsonObject.getJsonArray(FIELD_ANGULAR_VELOCITY), 3);

        int poseFrame = jsonObject.getInteger(FIELD_POSE_FRAME, 0);
        int velocityFrame = jsonObject.getInteger(FIELD_VELOCITY_FRAME, 0);

        return new VehicleOdometry(timestamp, timestampSample, position, q, velocity, angularVelocity, poseFrame, velocityFrame);
    }

    @Override
    public VehicleOdometry clone() {
        return new VehicleOdometry(this.timestamp, this.timestampSample,
                this.position.clone(), this.q.clone(), this.velocity.clone(),
                this.angularVelocity.clone(), this.poseFrame, this.velocityFrame);
    }

    // Helper interno per la conversione da JsonArray a float[]
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
