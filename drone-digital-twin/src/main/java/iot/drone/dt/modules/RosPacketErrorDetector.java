package iot.drone.dt.modules;
import io.vertx.core.json.DecodeException;
import io.vertx.core.json.JsonArray;
import io.vertx.core.json.JsonObject;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class RosPacketErrorDetector {

    /*
     * Stato per topic.
     *
     * Usiamo il topic come chiave perché odometry e status hanno sequence
     * indipendenti. Se usassi una chiave unica, i topic si disturberebbero
     * tra loro.
     */
    private final Map<String, Long> lastSequenceNumbers = new HashMap<>();
    private final Map<String, Long> lastTimestamps = new HashMap<>();

    /*
     * Soglie.
     *
     * Il forwarder, per simulare delay, sottrae circa 1000-5000 ms
     * dal timestamp del messaggio. Quindi se il timestamp torna indietro
     * più di 100 ms lo consideriamo delayed.
     */
    private static final long TIMESTAMP_BACKWARD_THRESHOLD_MICROS = 100_000L;

    /*
     * Salto massimo di sequenza accettato prima di considerarlo anomalo.
     * Questo intercetta buchi molto grandi. I drop piccoli invece non
     * vengono necessariamente classificati come OUT_OF_SEQUENCE.
     */
    private static final long MAX_SEQ_JUMP = 100;

    /*
     * Il forwarder genera CORRUPT_TS(future) con circa +10000 secondi.
     * Questa soglia serve a rilevarlo.
     *
     * Se le macchine sono molto desincronizzate, puoi aumentarla.
     * Deve comunque rimanere molto minore di 10000 se vuoi intercettare
     * la corruzione future del forwarder.
     */
    private static final long MAX_FUTURE_TIMESTAMP_SECONDS = 300L;

    /*
     * Range temporale robusto.
     *
     * Il forwarder genera:
     * - year2000 = 946684800000000 us
     * - year3000 = 32503680000000000 us
     *
     * Con year2020 come minimo, year2000 viene correttamente invalidato.
     */
    private static final long YEAR_2020_SECONDS = 1577836800L;
    private static final long YEAR_2100_SECONDS = 4102444800L;

    /*
     * Soglie coerenti con il forwarder:
     *
     * corrupt_data_odom:
     * - position[0] = 1e9f
     * - position[1] = -1e9f
     * - velocity[0] = 1e6f
     * - velocity[1] = -1e6f
     * - velocity[2] = 1e6f
     *
     * Quindi bisogna usare >=, non >.
     */
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

    // ============================================================
    // Interfaccia pubblica
    // ============================================================

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
        // 1. Validazioni base
        // ========================================================

        Long seq = getLongField(msg, "seq");
        if (seq == null) {
            return ValidationResult.CORRUPTED;
        }

        /*
         * Coerente con forwarder:
         * corrupt_header_* può impostare seq = 4294967295.
         */
        if (seq < 0 || seq == 4294967295L) {
            return ValidationResult.CORRUPTED;
        }

        Long timestamp = getLongField(msg, "timestamp");
        if (timestamp == null) {
            return ValidationResult.INVALID;
        }

        // ========================================================
        // 2. Validazione timestamp assoluta
        // ========================================================

        ValidationResult tsResult = validateTimestamp(timestamp);
        if (tsResult != ValidationResult.VALID) {
            return tsResult;
        }

        // ========================================================
        // 3. Validazioni specifiche messaggio
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
        // 4. Tracking sequenza/timestamp
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

        /*
         * Intercetta:
         * - CORRUPT_TS(year2000)
         * - CORRUPT_TS(year3000)
         */
        if (tsSecs < YEAR_2020_SECONDS || tsSecs >= YEAR_2100_SECONDS) {
            return ValidationResult.INVALID_TIMESTAMP;
        }

        /*
         * Intercetta:
         * - CORRUPT_TS(future)
         *
         * Il forwarder usa now + 10000 secondi.
         */
        if (tsSecs > nowSecs + MAX_FUTURE_TIMESTAMP_SECONDS) {
            return ValidationResult.INVALID_TIMESTAMP;
        }

        return ValidationResult.VALID;
    }

    // ============================================================
    // Sequenza + timestamp relativo
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

        /*
         * Aggiorniamo comunque lo stato per risincronizzarci.
         *
         * Nota: questa funzione viene chiamata solo dopo che timestamp
         * assoluto e campi strutturali sono risultati validi.
         */
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
        /*
         * Se il timestamp è uguale, il forwarder probabilmente sta
         * ripubblicando lo stesso ultimo messaggio PX4 con seq custom diverso.
         * Questo è normale nel tuo schema timer-based.
         */
        if (timestamp == lastTs) {
            if (seq < lastSeq) {
                return ValidationResult.OUT_OF_SEQUENCE;
            }
            return ValidationResult.VALID;
        }

        long deltaTs = timestamp - lastTs;

        /*
         * Coerente con make_delayed_odom/status:
         * il forwarder sottrae millisecondi dal timestamp.
         */
        if (deltaTs < -TIMESTAMP_BACKWARD_THRESHOLD_MICROS) {
            return ValidationResult.DELAYED_TIMESTAMP;
        }

        /*
         * Coerente con out_of_sequence:
         * un messaggio vecchio viene pubblicato dopo messaggi più nuovi,
         * quindi la seq può tornare indietro.
         */
        if (seq < lastSeq && seq != 0) {
            if (lastSeq > 4_000_000_000L && seq < 1000) {
                return ValidationResult.VALID;
            }

            return ValidationResult.OUT_OF_SEQUENCE;
        }

        /*
         * Salto enorme in avanti.
         * Questo può indicare perdita di molti messaggi o reset anomalo.
         */
        if (seq > lastSeq + MAX_SEQ_JUMP) {
            return ValidationResult.OUT_OF_SEQUENCE;
        }

        return ValidationResult.VALID;
    }

    // ============================================================
    // Odometry
    // ============================================================

    private ValidationResult validateOdometryFields(JsonObject msg) {
        /*
         * forwarder corrupt_header_odom:
         * - timestamp_sample tra 1000 e 2000
         */
        if (msg.containsKey("timestamp_sample")) {
            Long tsSample = getLongField(msg, "timestamp_sample");

            if (tsSample != null && tsSample >= 1000L && tsSample <= 2000L) {
                return ValidationResult.CORRUPTED;
            }
        }

        /*
         * forwarder corrupt_header_odom:
         * - pose_frame = 0
         * - velocity_frame = 0
         */
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

        /*
         * forwarder corrupt_data_odom:
         * - position NaN
         * - position huge +/-1e9
         */
        ValidationResult positionResult =
                validateNumericArray(msg, "position", MAX_ABS_POSITION, true);

        if (positionResult != ValidationResult.VALID) {
            return positionResult;
        }

        /*
         * forwarder corrupt_data_odom:
         * - bad_quaternion = [0,0,0,0]
         */
        ValidationResult qResult = validateQuaternion(msg);
        if (qResult != ValidationResult.VALID) {
            return qResult;
        }

        /*
         * forwarder corrupt_data_odom:
         * - velocity NaN
         * - velocity huge +/-1e6
         */
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

        /*
         * Anche le varianze dovrebbero essere finite se presenti.
         * Non imposto soglie strette, ma intercetto NaN/Inf.
         */
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

            /*
             * Il forwarder bad_quaternion produce norm = 0.
             */
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
        /*
         * forwarder corrupt_header_status:
         * - nav_state = 255
         */
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

        /*
         * forwarder corrupt_header_status:
         * - arming_state = 255
         */
        if (msg.containsKey("arming_state")) {
            Integer armingState = getIntegerField(msg, "arming_state");

            if (armingState == null) {
                return ValidationResult.CORRUPTED;
            }

            if (armingState == 255) {
                return ValidationResult.CORRUPTED;
            }

            /*
             * Non restringo troppo il range perché dipende dalla definizione
             * PX4 usata dalla tua versione. Intercetto il valore sentinella 255.
             */
        }

        /*
         * forwarder corrupt_data_status:
         * - hil_state = 255
         */
        if (msg.containsKey("hil_state")) {
            Integer hilState = getIntegerField(msg, "hil_state");

            if (hilState == null || hilState == 255) {
                return ValidationResult.CORRUPTED;
            }
        }

        /*
         * forwarder corrupt_data_status:
         * - vehicle_type = 255
         */
        if (msg.containsKey("vehicle_type")) {
            Integer vehicleType = getIntegerField(msg, "vehicle_type");

            if (vehicleType == null || vehicleType == 255) {
                return ValidationResult.CORRUPTED;
            }
        }

        /*
         * Questi campi booleani possono essere controllati solo per tipo.
         *
         * Se il forwarder li inverte, il pacchetto rimane formalmente valido:
         * failsafe = !failsafe è comunque un boolean.
         */
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
    // Helper JSON robusti
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
    // Utility pubbliche
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