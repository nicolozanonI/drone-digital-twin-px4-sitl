package iot.drone.dt.ros;

import com.google.gson.Gson;
import io.github.twinklekhj.ros.core.RosBridge;
import io.github.twinklekhj.ros.op.RosTopic;
import iot.drone.dt.ros.px4_msgs.OffboardControlMode;
import iot.drone.dt.ros.px4_msgs.TrajectorySetpoint;
import iot.drone.dt.ros.px4_msgs.VehicleCommand;
import iot.drone.dt.utils.Vector3D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class RosCommands extends Object{

    private final static Logger px4_conmmand_logger = LoggerFactory.getLogger(RosCommands.class);

    String droneId;

    int targetSystem;

    public RosCommands(String droneId, int targetSystem) {
        this.droneId = droneId;
        this.targetSystem = targetSystem;
    }

    OffboardControlMode offboardControlMode = new OffboardControlMode(0, true, false, false, false, false, false, false);

    float[] positionSetpoint = {0.0f, 0.0f, -5.0f};

    float[] velocityNaN = {Float.NaN, Float.NaN, Float.NaN};
    float[] accelerationNaN = {Float.NaN, Float.NaN, Float.NaN};
    float[] jerkNaN = {Float.NaN, Float.NaN, Float.NaN};

    float yawSetpoint = 0.0f;
    float yawSpeedSetpoint = 0.0f;

    public static String buildJsonPositionArrayMessage(float[] positionArray){
        try {
            Gson gson = new Gson();
            return gson.toJson(positionArray);
        }catch(Exception e) {
            e.printStackTrace();
            return null;
        }
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

    public static Vector3D parseFloatArray(byte[] payload) {

        try {
            Gson gson = new Gson();
            Vector3D array = gson.fromJson(new String(payload), Vector3D.class);
            return array;
        }catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void px4Arm(byte[] data, RosBridge rosBridge) throws InterruptedException {

        VehicleCommand vehicleCommand = new VehicleCommand(0, 1.0F, 0.0F, 0.0F,
                0.0F, 0.0F, 0.0F, 0.0F, 400, targetSystem, 1,
                1, 1, 0, true);

        RosTopic position_command_topic = RosTopic.builder('/' + droneId+"/fmu/in/vehicle_command", vehicleCommand).build();
        rosBridge.advertise(position_command_topic);
        rosBridge.publish(position_command_topic);
        Thread.sleep(50);
        rosBridge.publish(position_command_topic);

    }

    public void px4Land(byte[] data, RosBridge rosBridge) throws InterruptedException {
        VehicleCommand vehicleCommand = new VehicleCommand(0, 1.0F, 0.0F, 0.0F,
                0.0F, 0.0F, 0.0F, 0.0F, 21, targetSystem, 1,
                1, 1, 0, true);

        RosTopic position_command_topic = RosTopic.builder('/' + droneId+"/fmu/in/vehicle_command", vehicleCommand).build();
        rosBridge.advertise(position_command_topic);
        rosBridge.publish(position_command_topic);
        rosBridge.publish(position_command_topic);
    }

    public void px4Offboard(byte[] data, RosBridge rosBridge) {
        RosTopic offboardTopicMessage = RosTopic.builder('/' + droneId+"/fmu/in/offboard_control_mode", offboardControlMode).build();
        rosBridge.publish(offboardTopicMessage);
    }

    public void px4Takeoff(byte[] data, RosBridge rosBridge) throws InterruptedException {
        VehicleCommand vehicleCommand = new VehicleCommand(0, 1.0F, 6.0F, 0.0F,
                0.0F, 0.0F, 0.0F, 0.0F, 176, targetSystem, 1,
                1, 1, 0, true);
        RosTopic position_command_topic = RosTopic.builder('/' + droneId+"/fmu/in/vehicle_command", vehicleCommand).build();
        rosBridge.advertise(position_command_topic);
        rosBridge.publish(position_command_topic);
        Thread.sleep(50);
        rosBridge.publish(position_command_topic);
    }

    public void px4Disarm(byte[] data, RosBridge rosBridge) {
        VehicleCommand vehicleCommand = new VehicleCommand(0, 0.0F, 21196.0F, 0.0F,
                0.0F, 0.0F, 0.0F, 0.0F, 400, targetSystem, 1,
                1, 1, 0, true);
        RosTopic position_command_topic = RosTopic.builder('/' + droneId+"/fmu/in/vehicle_command", vehicleCommand).build();
        rosBridge.advertise(position_command_topic);
        rosBridge.publish(position_command_topic);
        rosBridge.publish(position_command_topic);
    }

    /*public void px4_stabilize(byte[] data, RosBridge rosBridge) {
        List<Object> set_mode = new ArrayList<>();
        set_mode.add((byte) 0);
        set_mode.add((String) "STABILIZED");
        RosService service = RosService.builder("mavros/set_mode", set_mode).build();
        rosBridge.callService(service, response -> {px4_conmmand_logger.info("{}", response.body().getJsonObject());});
        px4_conmmand_logger.info("{}", service.getArgs());
    }*/

    public void px4TrajectorySetopoint(byte[] data, RosBridge rosBridge) {
        Vector3D positionArray = parseMyArray(data);
        float[] positionFloat = {(float) positionArray.getX(), (float) positionArray.getY(), (float) positionArray.getZ()};

        TrajectorySetpoint trajectorySetpoint = new TrajectorySetpoint(
                0, // Timestamp in microseconds
                positionFloat,
                velocityNaN,
                accelerationNaN,
                jerkNaN,
                3.14F,
                yawSpeedSetpoint
        );
        RosTopic setpointTopicMessage = RosTopic.builder('/' + droneId + "/fmu/in/trajectory_setpoint", trajectorySetpoint).build();
        rosBridge.publish(setpointTopicMessage);
    }

}
