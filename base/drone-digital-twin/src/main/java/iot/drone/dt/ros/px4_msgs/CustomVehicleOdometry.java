package iot.drone.dt.ros.px4_msgs;


import io.github.twinklekhj.ros.type.RosMessage;
import io.vertx.core.json.JsonArray;
import io.vertx.core.json.JsonObject;
import iot.drone.dt.utils.Vector3D;
import lombok.Getter;
import lombok.ToString;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@ToString
@Getter
public class CustomVehicleOdometry extends RosMessage {
    public static final String TYPE = "px4_forwarder/msg/CustomVehicleOdometry";

    // ==================== FIELDS ====================
    public static final String FIELD_SEQUENCE = "seq";
    public static final String FIELD_TIMESTAMP = "timestamp";
    public static final String FIELD_TIMESTAMP_SAMPLE = "timestamp_sample";
    public static final String FIELD_POSITION = "position";
    public static final String FIELD_Q = "q";
    public static final String FIELD_VELOCITY = "velocity";
    public static final String FIELD_ANGULAR_VELOCITY = "angular_velocity";
    public static final String FIELD_POSE_FRAME = "pose_frame";
    public static final String FIELD_VELOCITY_FRAME = "velocity_frame";

    private int seq;
    private long timestamp;
    private long timestampSample;
    private float[] position;        // [x, y, z] in metri (NED)
    private float[] q;               // Quaternione [w, x, y, z]
    private float[] velocity;        // [vx, vy, vz] in m/s (NED)
    private float[] angularVelocity; // [rad/s]
    private int poseFrame;
    private int velocityFrame;

    public CustomVehicleOdometry() {
        this(0L, 0L, new float[3], new float[4], new float[3], new float[3], 0, 0, 200);
    }

    public CustomVehicleOdometry(long timestamp, long timestampSample, float[] position, float[] q,
                           float[] velocity, float[] angularVelocity, int poseFrame, int velocityFrame, int seq) {
        this.timestamp = timestamp;
        this.timestampSample = timestampSample;
        this.position = position;
        this.q = q;
        this.velocity = velocity;
        this.angularVelocity = angularVelocity;
        this.poseFrame = poseFrame;
        this.velocityFrame = velocityFrame;
        this.seq = seq;

        JsonObject obj = new JsonObject()
                .put(FIELD_TIMESTAMP, timestamp)
                .put(FIELD_TIMESTAMP_SAMPLE, timestampSample)
                .put(FIELD_POSITION, floatArrayToJsonArray(position))
                .put(FIELD_Q, floatArrayToJsonArray(q))
                .put(FIELD_VELOCITY, floatArrayToJsonArray(velocity))
                .put(FIELD_ANGULAR_VELOCITY, floatArrayToJsonArray(angularVelocity))
                .put(FIELD_POSE_FRAME, poseFrame)
                .put(FIELD_VELOCITY_FRAME, velocityFrame)
                .put(FIELD_SEQUENCE, seq);

        super.setJsonObject(obj);
        super.setType(TYPE);
    }

    public static CustomVehicleOdometry fromJsonString(String jsonString) {
        return CustomVehicleOdometry.fromMessage(new RosMessage(jsonString, TYPE));
    }

    public static CustomVehicleOdometry fromMessage(RosMessage m) {
        return CustomVehicleOdometry.fromJsonObject(m.getJsonObject());
    }

    public static CustomVehicleOdometry fromJsonObject(JsonObject jsonObject) {
        long timestamp = jsonObject.getLong(FIELD_TIMESTAMP, 0L);
        long timestampSample = jsonObject.getLong(FIELD_TIMESTAMP_SAMPLE, 0L);

        float[] position = toFloatArray(jsonObject.getJsonArray(FIELD_POSITION), 3);
        float[] q = toFloatArray(jsonObject.getJsonArray(FIELD_Q), 4);
        float[] velocity = toFloatArray(jsonObject.getJsonArray(FIELD_VELOCITY), 3);
        float[] angularVelocity = toFloatArray(jsonObject.getJsonArray(FIELD_ANGULAR_VELOCITY), 3);

        int poseFrame = jsonObject.getInteger(FIELD_POSE_FRAME, 0);
        int velocityFrame = jsonObject.getInteger(FIELD_VELOCITY_FRAME, 0);
        int seq = jsonObject.getInteger(FIELD_SEQUENCE, 0);

        return new CustomVehicleOdometry(timestamp, timestampSample, position, q, velocity, angularVelocity, poseFrame, velocityFrame, seq);
    }

    @Override
    public CustomVehicleOdometry clone() {
        return new CustomVehicleOdometry(this.timestamp, this.timestampSample,
                this.position.clone(), this.q.clone(), this.velocity.clone(),
                this.angularVelocity.clone(), this.poseFrame, this.velocityFrame, this.seq);
    }

    // Internal helper for converting a JsonArray to a float[]
    private static float[] toFloatArray(JsonArray array, int size) {
        float[] result = new float[size];
        if (array != null) {
            for (int i = 0; i < Math.min(size, array.size()); i++) {
                result[i] = array.getFloat(i);
            }
        }
        return result;
    }

    /**
     * Returns the column names for the CSV.
     * Array fields are expanded (e.g., position.x, position.y, position.z).
     */
    public List<String> getLabels() {
        return Arrays.asList(
                FIELD_SEQUENCE,
                FIELD_TIMESTAMP,
                FIELD_TIMESTAMP_SAMPLE,
                "pos_x", "pos_y", "pos_z",
                "q_w", "q_x", "q_y", "q_z",
                "vel_x", "vel_y", "vel_z",
                "ang_vel_x", "ang_vel_y", "ang_vel_z",
                FIELD_POSE_FRAME,
                FIELD_VELOCITY_FRAME
        );
    }

    /**
     * Returns the current values ready to be written to a CSV row.
     */
    public List<Object> getValues() {
        List<Object> values = new ArrayList<>();
        values.add(seq);
        values.add(timestamp);
        values.add(timestampSample);

        for (float f : position) values.add(f);
        for (float f : q) values.add(f);
        for (float f : velocity) values.add(f);
        for (float f : angularVelocity) values.add(f);

        values.add(poseFrame);
        values.add(velocityFrame);

        return values;
    }

    /**
     * Converts a primitive float[] array into a Vert.x JsonArray.
     */
    private static JsonArray floatArrayToJsonArray(float[] array) {
        JsonArray jsonArr = new JsonArray();
        if (array != null) {
            for (float v : array) {
                jsonArr.add(v);
            }
        }
        return jsonArr;
    }

    /**
     * Extracts the position and converts it into a Vector3D object.
     */
    public Vector3D getPositionAsArray() {
        if (this.position != null && this.position.length >= 3) {
            return new Vector3D(this.position[0], this.position[1], this.position[2]);
        }
        return new Vector3D(); // Ritorna (0,0,0) o NaN se preferisci
    }

    /**
     * Extracts the velocity and converts it into a Vector3D object.
     */
    public Vector3D getVelocityAsArray() {
        if (this.velocity != null && this.velocity.length >= 3) {
            return new Vector3D(this.velocity[0], this.velocity[1], this.velocity[2]);
        }
        return new Vector3D();
    }
}

