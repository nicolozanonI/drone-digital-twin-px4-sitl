package iot.drone.dt.ros.px4_msgs;

import io.github.twinklekhj.ros.type.RosMessage;
import io.vertx.core.json.JsonObject;
import lombok.Getter;
import lombok.ToString;

@ToString
@Getter
public class VehicleStatus extends RosMessage {
    public static final String TYPE = "px4_msgs/msg/VehicleStatus";

    // ==================== FIELD NAMES ====================
    public static final String FIELD_TIMESTAMP = "timestamp";
    public static final String FIELD_ARMING_STATE = "arming_state";
    public static final String FIELD_NAV_STATE = "nav_state";
    public static final String FIELD_FAILSAFE = "failsafe";
    public static final String FIELD_PRE_FLIGHT_CHECKS_PASS = "pre_flight_checks_pass";
    public static final String FIELD_VEHICLE_TYPE = "vehicle_type";
    public static final String FIELD_SYSTEM_ID = "system_id";
    public static final String FIELD_COMPONENT_ID = "component_id";
    public static final String FIELD_ARMING_REASON = "latest_arming_reason";
    public static final String FIELD_DISARMING_REASON = "latest_disarming_reason";
    public static final String FIELD_FAILSAFE_DEFER_STATE = "failsafe_defer_state";
    public static final String FIELD_HIL_STATE = "hil_state";

    // ==================== ARMING STATES ====================
    public static final int ARMING_STATE_DISARMED = 1;
    public static final int ARMING_STATE_ARMED = 2;

    // ==================== ARM/DISARM REASONS ====================
    public static final int ARM_DISARM_REASON_STICK_GESTURE = 1;
    public static final int ARM_DISARM_REASON_RC_SWITCH = 2;
    public static final int ARM_DISARM_REASON_COMMAND_INTERNAL = 3;
    public static final int ARM_DISARM_REASON_COMMAND_EXTERNAL = 4;
    public static final int ARM_DISARM_REASON_MISSION_START = 5;
    public static final int ARM_DISARM_REASON_LANDING = 6;
    public static final int ARM_DISARM_REASON_PREFLIGHT_INACTION = 7;
    public static final int ARM_DISARM_REASON_KILL_SWITCH = 8;
    public static final int ARM_DISARM_REASON_RC_BUTTON = 13;
    public static final int ARM_DISARM_REASON_FAILSAFE = 14;

    // ==================== NAVIGATION STATES ====================
    public static final int NAVIGATION_STATE_MANUAL = 0;
    public static final int NAVIGATION_STATE_ALTCTL = 1;
    public static final int NAVIGATION_STATE_POSCTL = 2;
    public static final int NAVIGATION_STATE_AUTO_MISSION = 3;
    public static final int NAVIGATION_STATE_AUTO_LOITER = 4;
    public static final int NAVIGATION_STATE_AUTO_RTL = 5;
    public static final int NAVIGATION_STATE_POSITION_SLOW = 6;
    public static final int NAVIGATION_STATE_FREE5 = 7;
    public static final int NAVIGATION_STATE_ALTITUDE_CRUISE = 8;
    public static final int NAVIGATION_STATE_FREE3 = 9;
    public static final int NAVIGATION_STATE_ACRO = 10;
    public static final int NAVIGATION_STATE_FREE2 = 11;
    public static final int NAVIGATION_STATE_DESCEND = 12;
    public static final int NAVIGATION_STATE_TERMINATION = 13;
    public static final int NAVIGATION_STATE_OFFBOARD = 14;
    public static final int NAVIGATION_STATE_STAB = 15;
    public static final int NAVIGATION_STATE_FREE1 = 16;
    public static final int NAVIGATION_STATE_AUTO_TAKEOFF = 17;
    public static final int NAVIGATION_STATE_AUTO_LAND = 18;
    public static final int NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19;
    public static final int NAVIGATION_STATE_AUTO_PRECLAND = 20;
    public static final int NAVIGATION_STATE_ORBIT = 21;
    public static final int NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22;
    public static final int NAVIGATION_STATE_EXTERNAL1 = 23;
    public static final int NAVIGATION_STATE_EXTERNAL2 = 24;
    public static final int NAVIGATION_STATE_EXTERNAL3 = 25;
    public static final int NAVIGATION_STATE_EXTERNAL4 = 26;
    public static final int NAVIGATION_STATE_EXTERNAL5 = 27;
    public static final int NAVIGATION_STATE_EXTERNAL6 = 28;
    public static final int NAVIGATION_STATE_EXTERNAL7 = 29;
    public static final int NAVIGATION_STATE_EXTERNAL8 = 30;
    public static final int NAVIGATION_STATE_MAX = 31;

    // ==================== VEHICLE TYPES ====================
    public static final int VEHICLE_TYPE_UNSPECIFIED = 0;
    public static final int VEHICLE_TYPE_ROTARY_WING = 1;
    public static final int VEHICLE_TYPE_FIXED_WING = 2;
    public static final int VEHICLE_TYPE_ROVER = 3;

    // ==================== FAILSAFE DEFER STATES ====================
    public static final int FAILSAFE_DEFER_STATE_DISABLED = 0;
    public static final int FAILSAFE_DEFER_STATE_ENABLED = 1;
    public static final int FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2;

    // ==================== HIL STATES ====================
    public static final int HIL_STATE_OFF = 0;
    public static final int HIL_STATE_ON = 1;

    // ==================== FIELDS ====================
    private long timestamp;
    private int armingState;
    private int navState;
    private boolean failsafe;
    private boolean preFlightChecksPass;
    private int vehicleType;
    private int systemId;
    private int componentId;

    // Campi aggiuntivi utili per il debug
    private int latestArmingReason;
    private int latestDisarmingReason;
    private int failsafeDeferState;
    private int hilState;

    // ==================== COSTRUTTORI ====================
    public VehicleStatus() {
        this(0, 0, 0, false, false, 0, 0, 0);
    }

    public VehicleStatus(long timestamp, int armingState, int navState, boolean failsafe,
                         boolean preFlightChecksPass, int vehicleType, int systemId, int componentId) {
        this(timestamp, armingState, navState, failsafe, preFlightChecksPass, vehicleType,
                systemId, componentId, 0, 0, 0, 0);
    }

    public VehicleStatus(long timestamp, int armingState, int navState, boolean failsafe,
                         boolean preFlightChecksPass, int vehicleType, int systemId, int componentId,
                         int latestArmingReason, int latestDisarmingReason, int failsafeDeferState, int hilState) {
        this.timestamp = timestamp;
        this.armingState = armingState;
        this.navState = navState;
        this.failsafe = failsafe;
        this.preFlightChecksPass = preFlightChecksPass;
        this.vehicleType = vehicleType;
        this.systemId = systemId;
        this.componentId = componentId;
        this.latestArmingReason = latestArmingReason;
        this.latestDisarmingReason = latestDisarmingReason;
        this.failsafeDeferState = failsafeDeferState;
        this.hilState = hilState;

        JsonObject obj = new JsonObject()
                .put(FIELD_TIMESTAMP, timestamp)
                .put(FIELD_ARMING_STATE, armingState)
                .put(FIELD_NAV_STATE, navState)
                .put(FIELD_FAILSAFE, failsafe)
                .put(FIELD_PRE_FLIGHT_CHECKS_PASS, preFlightChecksPass)
                .put(FIELD_VEHICLE_TYPE, vehicleType)
                .put(FIELD_SYSTEM_ID, systemId)
                .put(FIELD_COMPONENT_ID, componentId)
                .put(FIELD_ARMING_REASON, latestArmingReason)
                .put(FIELD_DISARMING_REASON, latestDisarmingReason)
                .put(FIELD_FAILSAFE_DEFER_STATE, failsafeDeferState)
                .put(FIELD_HIL_STATE, hilState);

        super.setJsonObject(obj);
        super.setType(TYPE);
    }

    // ==================== STATIC HELPER METHODS (int → String) ====================

    public static String getArmingStateString(int state) {
        return switch (state) {
            case ARMING_STATE_DISARMED -> "DISARMED";
            case ARMING_STATE_ARMED -> "ARMED";
            default -> "UNKNOWN_ARMING_STATE(" + state + ")";
        };
    }

    public static String getNavStateString(int state) {
        return switch (state) {
            case NAVIGATION_STATE_MANUAL -> "MANUAL";
            case NAVIGATION_STATE_ALTCTL -> "ALTCTL";
            case NAVIGATION_STATE_POSCTL -> "POSCTL";
            case NAVIGATION_STATE_AUTO_MISSION -> "AUTO_MISSION";
            case NAVIGATION_STATE_AUTO_LOITER -> "AUTO_LOITER";
            case NAVIGATION_STATE_AUTO_RTL -> "AUTO_RTL";
            case NAVIGATION_STATE_POSITION_SLOW -> "POSITION_SLOW";
            case NAVIGATION_STATE_ALTITUDE_CRUISE -> "ALTITUDE_CRUISE";
            case NAVIGATION_STATE_ACRO -> "ACRO";
            case NAVIGATION_STATE_DESCEND -> "DESCEND";
            case NAVIGATION_STATE_TERMINATION -> "TERMINATION";
            case NAVIGATION_STATE_OFFBOARD -> "OFFBOARD";
            case NAVIGATION_STATE_STAB -> "STABILIZED";
            case NAVIGATION_STATE_AUTO_TAKEOFF -> "AUTO_TAKEOFF";
            case NAVIGATION_STATE_AUTO_LAND -> "AUTO_LAND";
            case NAVIGATION_STATE_AUTO_FOLLOW_TARGET -> "AUTO_FOLLOW_TARGET";
            case NAVIGATION_STATE_AUTO_PRECLAND -> "AUTO_PRECLAND";
            case NAVIGATION_STATE_ORBIT -> "ORBIT";
            case NAVIGATION_STATE_AUTO_VTOL_TAKEOFF -> "AUTO_VTOL_TAKEOFF";
            case NAVIGATION_STATE_EXTERNAL1 -> "EXTERNAL1";
            case NAVIGATION_STATE_EXTERNAL2 -> "EXTERNAL2";
            case NAVIGATION_STATE_EXTERNAL3 -> "EXTERNAL3";
            case NAVIGATION_STATE_EXTERNAL4 -> "EXTERNAL4";
            case NAVIGATION_STATE_EXTERNAL5 -> "EXTERNAL5";
            case NAVIGATION_STATE_EXTERNAL6 -> "EXTERNAL6";
            case NAVIGATION_STATE_EXTERNAL7 -> "EXTERNAL7";
            case NAVIGATION_STATE_EXTERNAL8 -> "EXTERNAL8";
            default -> "UNKNOWN_NAV_STATE(" + state + ")";
        };
    }

    public static String getVehicleTypeString(int type) {
        return switch (type) {
            case VEHICLE_TYPE_UNSPECIFIED -> "UNSPECIFIED";
            case VEHICLE_TYPE_ROTARY_WING -> "ROTARY_WING";
            case VEHICLE_TYPE_FIXED_WING -> "FIXED_WING";
            case VEHICLE_TYPE_ROVER -> "ROVER";
            default -> "UNKNOWN_VEHICLE_TYPE(" + type + ")";
        };
    }

    public static String getArmDisarmReasonString(int reason) {
        return switch (reason) {
            case 0 -> "NONE";
            case ARM_DISARM_REASON_STICK_GESTURE -> "STICK_GESTURE";
            case ARM_DISARM_REASON_RC_SWITCH -> "RC_SWITCH";
            case ARM_DISARM_REASON_COMMAND_INTERNAL -> "COMMAND_INTERNAL";
            case ARM_DISARM_REASON_COMMAND_EXTERNAL -> "COMMAND_EXTERNAL";
            case ARM_DISARM_REASON_MISSION_START -> "MISSION_START";
            case ARM_DISARM_REASON_LANDING -> "LANDING";
            case ARM_DISARM_REASON_PREFLIGHT_INACTION -> "PREFLIGHT_INACTION";
            case ARM_DISARM_REASON_KILL_SWITCH -> "KILL_SWITCH";
            case ARM_DISARM_REASON_RC_BUTTON -> "RC_BUTTON";
            case ARM_DISARM_REASON_FAILSAFE -> "FAILSAFE";
            default -> "UNKNOWN_REASON(" + reason + ")";
        };
    }

    public static String getFailsafeDeferStateString(int state) {
        return switch (state) {
            case FAILSAFE_DEFER_STATE_DISABLED -> "DISABLED";
            case FAILSAFE_DEFER_STATE_ENABLED -> "ENABLED";
            case FAILSAFE_DEFER_STATE_WOULD_FAILSAFE -> "WOULD_FAILSAFE";
            default -> "UNKNOWN_FAILSAFE_DEFER_STATE(" + state + ")";
        };
    }

    public static String getHilStateString(int state) {
        return switch (state) {
            case HIL_STATE_OFF -> "OFF";
            case HIL_STATE_ON -> "ON";
            default -> "UNKNOWN_HIL_STATE(" + state + ")";
        };
    }

    // ==================== INSTANCE HELPER METHODS ====================

    public String getArmingStateDescription() {
        return getArmingStateString(this.armingState);
    }

    public String getNavStateDescription() {
        return getNavStateString(this.navState);
    }

    public String getVehicleTypeDescription() {
        return getVehicleTypeString(this.vehicleType);
    }

    public String getLatestArmingReasonDescription() {
        return getArmDisarmReasonString(this.latestArmingReason);
    }

    public String getLatestDisarmingReasonDescription() {
        return getArmDisarmReasonString(this.latestDisarmingReason);
    }

    public String getFailsafeDeferStateDescription() {
        return getFailsafeDeferStateString(this.failsafeDeferState);
    }

    public String getHilStateDescription() {
        return getHilStateString(this.hilState);
    }

    // ==================== DETAILED STRING REPRESENTATION ====================

    /**
     * Restituisce una rappresentazione leggibile dello stato del veicolo,
     * utile per logging e debug.
     */
    public String toDetailedString() {
        return String.format("""
            VehicleStatus {
              timestamp: %d ms,
              arming_state: %s (%d),
              nav_state: %s (%d),
              vehicle_type: %s (%d),
              failsafe: %b,
              pre_flight_checks_pass: %b,
              system_id: %d, component_id: %d,
              latest_arming_reason: %s,
              latest_disarming_reason: %s,
              failsafe_defer_state: %s,
              hil_state: %s
            }""",
                timestamp / 1000, // conversione da µs a ms per leggibilità
                getArmingStateDescription(), armingState,
                getNavStateDescription(), navState,
                getVehicleTypeDescription(), vehicleType,
                failsafe, preFlightChecksPass,
                systemId, componentId,
                getLatestArmingReasonDescription(),
                getLatestDisarmingReasonDescription(),
                getFailsafeDeferStateDescription(),
                getHilStateDescription()
        );
    }

    // ==================== METODI DI PARSING JSON ====================

    public static VehicleStatus fromJsonString(String jsonString) {
        return VehicleStatus.fromMessage(new RosMessage(jsonString, TYPE));
    }

    public static VehicleStatus fromMessage(RosMessage m) {
        return VehicleStatus.fromJsonObject(m.getJsonObject());
    }

    public static VehicleStatus fromJsonObject(JsonObject jsonObject) {
        long timestamp = jsonObject.getLong(FIELD_TIMESTAMP, 0L);
        int armingState = jsonObject.getInteger(FIELD_ARMING_STATE, 0);
        int navState = jsonObject.getInteger(FIELD_NAV_STATE, 0);
        boolean failsafe = jsonObject.getBoolean(FIELD_FAILSAFE, false);
        boolean preFlightChecksPass = jsonObject.getBoolean(FIELD_PRE_FLIGHT_CHECKS_PASS, false);
        int vehicleType = jsonObject.getInteger(FIELD_VEHICLE_TYPE, 0);
        int systemId = jsonObject.getInteger(FIELD_SYSTEM_ID, 0);
        int componentId = jsonObject.getInteger(FIELD_COMPONENT_ID, 0);
        int latestArmingReason = jsonObject.getInteger(FIELD_ARMING_REASON, 0);
        int latestDisarmingReason = jsonObject.getInteger(FIELD_DISARMING_REASON, 0);
        int failsafeDeferState = jsonObject.getInteger(FIELD_FAILSAFE_DEFER_STATE, 0);
        int hilState = jsonObject.getInteger(FIELD_HIL_STATE, 0);

        return new VehicleStatus(timestamp, armingState, navState, failsafe,
                preFlightChecksPass, vehicleType, systemId, componentId,
                latestArmingReason, latestDisarmingReason, failsafeDeferState, hilState);
    }

    @Override
    public VehicleStatus clone() {
        return new VehicleStatus(this.timestamp, this.armingState, this.navState,
                this.failsafe, this.preFlightChecksPass, this.vehicleType,
                this.systemId, this.componentId,
                this.latestArmingReason, this.latestDisarmingReason,
                this.failsafeDeferState, this.hilState);
    }
}

/*
// Esempio di utilizzo
VehicleStatus status = VehicleStatus.fromJsonString(jsonString);

// Ottenere descrizioni leggibili
System.out.println("Stato armamento: " + status.getArmingStateDescription());
// Output: "ARMED"

System.out.println("Modalità navigazione: " + status.getNavStateDescription());
// Output: "OFFBOARD"

// Output completo per debug
System.out.println(status.toDetailedString());
 */