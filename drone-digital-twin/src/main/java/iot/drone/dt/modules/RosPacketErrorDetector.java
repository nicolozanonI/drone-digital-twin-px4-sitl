package iot.drone.dt.modules;
import io.vertx.core.json.DecodeException;
import io.vertx.core.json.JsonArray;
import io.vertx.core.json.JsonObject;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class RosPacketErrorDetector {

    private final Map<String, Long> lastSequenceNumbers = new HashMap<>();
    private final Map<String, Long> lastTimestamps = new HashMap<>();

    private static final long TIMESTAMP_BACKWARD_THRESHOLD_MICROS = 100_000L;

    private static final long MAX_SEQ_JUMP = 100;

    private static final long MAX_FUTURE_TIMESTAMP_SECONDS = 300L;

    private static final long YEAR_2020_SECONDS = 1577836800L;
    private static final long YEAR_2100_SECONDS = 4102444800L;

    private static final double MAX_ABS_POSITION = 1e9;
    private static final double MAX_ABS_VELOCITY = 1e6;

    private static final Set<String> ODOMETRY_FIELDS = Set.of(
            "position",
            "q",
            "velocity",
            "angular_velocity",
            "timestamp_sample",
            "pose_frame",
            "velocity_frame",
            "position_variance",
            "orientation_variance",
            "velocity_variance"
    );

    private static final Set<String> STATUS_FIELDS = Set.of(
            "nav_state",
            "arming_state",
            "failsafe",
            "pre_flight_checks_pass",
            "safety_off",
            "hil_state",
            "vehicle_type",
            "gcs_connection_lost",
            "in_transition_mode"
    );

    public enum ValidationResult {
        VALID,
        INVALID,
        CORRUPTED,
        OUT_OF_SEQUENCE,
        DELAYED_TIMESTAMP,
        INVALID_TIMESTAMP,
        INVALID_STATE_VALUE
    }

    public boolean hasProblems(String raw, String topic) {
        return validateMessage(raw, topic) != ValidationResult.VALID;
    }

    public boolean hasProblems(JsonObject json, String topic) {
        return validateMessage(json, topic) != ValidationResult.VALID;
    }

    public boolean isValid(String raw, String topic) {
        return validateMessage(raw, topic) == ValidationResult.VALID;
    }

    public boolean isValid(JsonObject json, String topic) {
        return validateMessage(json, topic) == ValidationResult.VALID;
    }

    public ValidationResult validateMessage(String rawMessage, String topicName) {
        try {
            return validateMessage(new JsonObject(rawMessage), topicName);
        } catch (DecodeException e) {
            return ValidationResult.INVALID;
        } catch (Exception e) {
            return ValidationResult.INVALID;
        }
    }

    public ValidationResult validateMessage(JsonObject msg, String topicName) {
        if (msg == null || topicName == null || topicName.isBlank()) {
            return ValidationResult.INVALID;
        }

        // ========================================================
        // 1. Sequence number and Timestamp corruption detection
        // ========================================================

        Long seq = getLongField(msg, "seq");
        if (seq == null) {
            return ValidationResult.CORRUPTED;
        }

        if (seq < 0 || seq == 4294967295L) {
            return ValidationResult.CORRUPTED;
        }

        Long timestamp = getLongField(msg, "timestamp");
        if (timestamp == null) {
            return ValidationResult.INVALID;
        }

        // ========================================================
        // 2. Timestamp validation
        // ========================================================

        ValidationResult tsResult = validateTimestamp(timestamp);
        if (tsResult != ValidationResult.VALID) {
            return tsResult;
        }

        // ========================================================
        // 3. Message fields corruption detection
        // ========================================================

        if (hasAnyField(msg, ODOMETRY_FIELDS)) {
            ValidationResult r = validateOdometryFields(msg);
            if (r != ValidationResult.VALID) {
                return r;
            }
        }

        if (hasAnyField(msg, STATUS_FIELDS)) {
            ValidationResult r = validateStatusFields(msg);
            if (r != ValidationResult.VALID) {
                return r;
            }
        }

        // ========================================================
        // 4. Tracking sequence/timestamp
        // ========================================================

        ValidationResult seqResult =
                validateSequenceWithTimestamp(seq, timestamp, topicName);

        if (seqResult != ValidationResult.VALID) {
            return seqResult;
        }

        return ValidationResult.VALID;
    }

    // ============================================================
    // Timestamp
    // ============================================================

    private ValidationResult validateTimestamp(long timestampMicros) {
        if (timestampMicros <= 0) {
            return ValidationResult.INVALID_TIMESTAMP;
        }

        long tsSecs = timestampMicros / 1_000_000L;
        long nowSecs = Instant.now().getEpochSecond();

        if (tsSecs < YEAR_2020_SECONDS || tsSecs >= YEAR_2100_SECONDS) {
            return ValidationResult.INVALID_TIMESTAMP;
        }

        if (tsSecs > nowSecs + MAX_FUTURE_TIMESTAMP_SECONDS) {
            return ValidationResult.INVALID_TIMESTAMP;
        }

        return ValidationResult.VALID;
    }

    // ============================================================
    // Method to validate sequence + timestamp
    // ============================================================

    private ValidationResult validateSequenceWithTimestamp(
            long seq,
            long timestamp,
            String topicName
    ) {
        if (!lastSequenceNumbers.containsKey(topicName)) {
            lastSequenceNumbers.put(topicName, seq);
            lastTimestamps.put(topicName, timestamp);
            return ValidationResult.VALID;
        }

        long lastSeq = lastSequenceNumbers.get(topicName);
        long lastTs = lastTimestamps.get(topicName);

        ValidationResult result =
                computeSequenceResult(seq, timestamp, lastSeq, lastTs);

        lastSequenceNumbers.put(topicName, seq);
        lastTimestamps.put(topicName, timestamp);

        return result;
    }

    private ValidationResult computeSequenceResult(
            long seq,
            long timestamp,
            long lastSeq,
            long lastTs
    ) {
        if (timestamp == lastTs) {
            if (seq < lastSeq) {
                return ValidationResult.OUT_OF_SEQUENCE;
            }
            return ValidationResult.VALID;
        }

        long deltaTs = timestamp - lastTs;

        if (deltaTs < -TIMESTAMP_BACKWARD_THRESHOLD_MICROS) {
            return ValidationResult.DELAYED_TIMESTAMP;
        }

        if (seq < lastSeq && seq != 0) {
            if (lastSeq > 4_000_000_000L && seq < 1000) {
                return ValidationResult.VALID;
            }

            return ValidationResult.OUT_OF_SEQUENCE;
        }

        if (seq > lastSeq + MAX_SEQ_JUMP) {
            return ValidationResult.OUT_OF_SEQUENCE;
        }

        return ValidationResult.VALID;
    }

    // ============================================================
    // Odometry
    // ============================================================

    private ValidationResult validateOdometryFields(JsonObject msg) {

        if (msg.containsKey("timestamp_sample")) {
            Long tsSample = getLongField(msg, "timestamp_sample");

            if (tsSample != null && tsSample >= 1000L && tsSample <= 2000L) {
                return ValidationResult.CORRUPTED;
            }
        }

        if (msg.containsKey("pose_frame")) {
            Integer poseFrame = getIntegerField(msg, "pose_frame");

            if (poseFrame == null || poseFrame == 0) {
                return ValidationResult.CORRUPTED;
            }
        }

        if (msg.containsKey("velocity_frame")) {
            Integer velocityFrame = getIntegerField(msg, "velocity_frame");

            if (velocityFrame == null || velocityFrame == 0) {
                return ValidationResult.CORRUPTED;
            }
        }

        ValidationResult positionResult =
                validateNumericArray(msg, "position", MAX_ABS_POSITION, true);

        if (positionResult != ValidationResult.VALID) {
            return positionResult;
        }

        ValidationResult qResult = validateQuaternion(msg);
        if (qResult != ValidationResult.VALID) {
            return qResult;
        }

        ValidationResult velocityResult =
                validateNumericArray(msg, "velocity", MAX_ABS_VELOCITY, true);

        if (velocityResult != ValidationResult.VALID) {
            return velocityResult;
        }

        ValidationResult angularVelocityResult =
                validateNumericArray(msg, "angular_velocity", MAX_ABS_VELOCITY, true);

        if (angularVelocityResult != ValidationResult.VALID) {
            return angularVelocityResult;
        }

        for (String field : new String[]{
                "position_variance",
                "orientation_variance",
                "velocity_variance"
        }) {
            ValidationResult varianceResult =
                    validateNumericArray(msg, field, Double.POSITIVE_INFINITY, false);

            if (varianceResult != ValidationResult.VALID) {
                return varianceResult;
            }
        }

        return ValidationResult.VALID;
    }

    private ValidationResult validateQuaternion(JsonObject msg) {
        if (!msg.containsKey("q")) {
            return ValidationResult.VALID;
        }

        try {
            JsonArray q = msg.getJsonArray("q");

            if (q == null || q.size() != 4) {
                return ValidationResult.CORRUPTED;
            }

            double normSq = 0.0;

            for (int i = 0; i < 4; i++) {
                Double v = getDoubleFromArray(q, i);

                if (v == null || Double.isNaN(v) || Double.isInfinite(v)) {
                    return ValidationResult.CORRUPTED;
                }

                normSq += v * v;
            }

            double norm = Math.sqrt(normSq);

            if (Math.abs(norm - 1.0) > 0.15) {
                return ValidationResult.CORRUPTED;
            }

            return ValidationResult.VALID;

        } catch (Exception e) {
            return ValidationResult.CORRUPTED;
        }
    }

    private ValidationResult validateNumericArray(
            JsonObject msg,
            String fieldName,
            double maxAbsAllowed,
            boolean checkThresholdInclusive
    ) {
        if (!msg.containsKey(fieldName)) {
            return ValidationResult.VALID;
        }

        try {
            JsonArray arr = msg.getJsonArray(fieldName);

            if (arr == null) {
                return ValidationResult.CORRUPTED;
            }

            for (int i = 0; i < arr.size(); i++) {
                Double v = getDoubleFromArray(arr, i);

                if (v == null || Double.isNaN(v) || Double.isInfinite(v)) {
                    return ValidationResult.CORRUPTED;
                }

                if (Double.isFinite(maxAbsAllowed)) {
                    if (checkThresholdInclusive) {
                        if (Math.abs(v) >= maxAbsAllowed) {
                            return ValidationResult.CORRUPTED;
                        }
                    } else {
                        if (Math.abs(v) > maxAbsAllowed) {
                            return ValidationResult.CORRUPTED;
                        }
                    }
                }
            }

            return ValidationResult.VALID;

        } catch (Exception e) {
            return ValidationResult.CORRUPTED;
        }
    }

    // ============================================================
    // Status
    // ============================================================

    private ValidationResult validateStatusFields(JsonObject msg) {

        if (msg.containsKey("nav_state")) {
            Integer navState = getIntegerField(msg, "nav_state");

            if (navState == null) {
                return ValidationResult.CORRUPTED;
            }

            if (navState == 255) {
                return ValidationResult.CORRUPTED;
            }

            if (navState < 0 || navState > 31) {
                return ValidationResult.INVALID_STATE_VALUE;
            }
        }


        if (msg.containsKey("arming_state")) {
            Integer armingState = getIntegerField(msg, "arming_state");

            if (armingState == null) {
                return ValidationResult.CORRUPTED;
            }

            if (armingState == 255) {
                return ValidationResult.CORRUPTED;
            }

        }

        if (msg.containsKey("hil_state")) {
            Integer hilState = getIntegerField(msg, "hil_state");

            if (hilState == null || hilState == 255) {
                return ValidationResult.CORRUPTED;
            }
        }
        if (msg.containsKey("vehicle_type")) {
            Integer vehicleType = getIntegerField(msg, "vehicle_type");

            if (vehicleType == null || vehicleType == 255) {
                return ValidationResult.CORRUPTED;
            }
        }

        for (String f : new String[]{
                "failsafe",
                "pre_flight_checks_pass",
                "safety_off",
                "gcs_connection_lost",
                "in_transition_mode"
        }) {
            if (msg.containsKey(f)) {
                Object v = msg.getValue(f);

                if (v != null && !(v instanceof Boolean)) {
                    return ValidationResult.CORRUPTED;
                }
            }
        }

        return ValidationResult.VALID;
    }

    // ============================================================
    // Helper JSON
    // ============================================================

    private boolean hasAnyField(JsonObject msg, Set<String> fields) {
        for (String f : fields) {
            if (msg.containsKey(f)) {
                return true;
            }
        }

        return false;
    }

    private Long getLongField(JsonObject msg, String field) {
        Object value = msg.getValue(field);

        if (value == null) {
            return null;
        }

        if (value instanceof Number n) {
            return n.longValue();
        }

        if (value instanceof String s) {
            try {
                return Long.parseLong(s);
            } catch (NumberFormatException e) {
                return null;
            }
        }

        return null;
    }

    private Integer getIntegerField(JsonObject msg, String field) {
        Object value = msg.getValue(field);

        if (value == null) {
            return null;
        }

        if (value instanceof Number n) {
            return n.intValue();
        }

        if (value instanceof String s) {
            try {
                return Integer.parseInt(s);
            } catch (NumberFormatException e) {
                return null;
            }
        }

        return null;
    }

    private Double getDoubleFromArray(JsonArray arr, int index) {
        Object value = arr.getValue(index);

        if (value == null) {
            return null;
        }

        if (value instanceof Number n) {
            return n.doubleValue();
        }

        if (value instanceof String s) {
            try {
                return Double.parseDouble(s);
            } catch (NumberFormatException e) {
                return null;
            }
        }

        return null;
    }

    // ============================================================
    // Utils
    // ============================================================

    public String messageValidationResult(String raw, String topic) {
        return formatResult(validateMessage(raw, topic));
    }

    public String messageValidationResult(JsonObject json, String topic) {
        return formatResult(validateMessage(json, topic));
    }

    private String formatResult(ValidationResult r) {
        return switch (r) {
            case VALID -> "VALID";
            case INVALID -> "INVALID";
            case OUT_OF_SEQUENCE -> "OUT_OF_SEQUENCE";
            case DELAYED_TIMESTAMP -> "DELAYED_TIMESTAMP";
            case INVALID_TIMESTAMP -> "INVALID_TIMESTAMP";
            case CORRUPTED -> "CORRUPTED";
            case INVALID_STATE_VALUE -> "INVALID_STATE_VALUE";
        };
    }

    public void resetSequenceTracking(String topic) {
        lastSequenceNumbers.remove(topic);
        lastTimestamps.remove(topic);
    }

    public void resetAllSequenceTracking() {
        lastSequenceNumbers.clear();
        lastTimestamps.clear();
    }

    public Long getLastSequenceNumber(String topic) {
        return lastSequenceNumbers.get(topic);
    }

    public Long getLastTimestamp(String topic) {
        return lastTimestamps.get(topic);
    }

    // ============================================================
    // Debug utility
    // ============================================================

    public String getValidationDebug(JsonObject msg, String topic) {
        StringBuilder sb = new StringBuilder();

        Long seq = getLongField(msg, "seq");
        Long ts = getLongField(msg, "timestamp");

        Long lastSeq = lastSequenceNumbers.get(topic);
        Long lastTs = lastTimestamps.get(topic);

        sb.append("Topic: ").append(topic)
                .append(" | seq=").append(seq)
                .append(" | ts=").append(ts);

        if (lastSeq != null) {
            sb.append(" | lastSeq=").append(lastSeq);
        }

        if (lastTs != null) {
            sb.append(" | lastTs=").append(lastTs);
        }

        if (lastTs != null && ts != null) {
            long delta = ts - lastTs;
            sb.append(" | deltaTs=").append(delta).append("us");

            if (delta == 0) {
                sb.append(" [FORWARDER_REPEAT]");
            }

            if (delta < -TIMESTAMP_BACKWARD_THRESHOLD_MICROS) {
                sb.append(" [DELAYED_TIMESTAMP_CANDIDATE]");
            }
        }

        ValidationResult result = validateMessage(msg, topic);
        sb.append(" | result=").append(formatResult(result));

        return sb.toString();
    }
}