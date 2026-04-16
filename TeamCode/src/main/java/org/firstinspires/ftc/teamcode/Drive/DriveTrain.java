package org.firstinspires.ftc.teamcode.Drive;

import android.util.Log;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

//TODO: second order kinematics, account for wheel turning time (can be done last)
public class DriveTrain {
    public static final double MOTOR_MAX_RPM = 1150.0; // free speed of each drive motor
    public static final double WHEEL_DIAMETER_IN = 3.0;    // diff swerve wheel diameter
    public static final double DRIVE_GEAR_RATIO = 1.0;    // wheel rev per motor rev (differential path)
    private double verticalModuleOffset = 10;
    private double horizontalModuleOffset = 10;
    private List<Vector2d> modulePoses = new ArrayList<>();
    private Module leftModule;
    private Module rightModule;
    List<LynxModule> allhubs;
    public final double MAX_INCHES_PER_SEC =
            (MOTOR_MAX_RPM * DRIVE_GEAR_RATIO / 60.0) * Math.PI * WHEEL_DIAMETER_IN;
    // changes as you go more and more out, you need more and more speed to move
    //fastest the robot is allowed to spin cuz any faster would mean farthest wheel can't keep up
    public final double MAX_RAD_PER_SEC =
            MAX_INCHES_PER_SEC /(Math.hypot(horizontalModuleOffset, verticalModuleOffset));

    public DriveTrain(HardwareMap hwmap) {
        modulePoses.add(new Vector2d(horizontalModuleOffset, verticalModuleOffset)); // left module pose
        modulePoses.add(new Vector2d(-horizontalModuleOffset, verticalModuleOffset)); // right module pose

        leftModule = new Module(hwmap, "motor1", "motor2", "sensor1");
        rightModule = new Module(hwmap, "motor3", "motor4", "sensor2");

        allhubs = hwmap.getAll(LynxModule.class);
        for(LynxModule hub : allhubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

        //angular rotation is in rad/s
    // velocityvector in in/s
    public void update(Vector2d velocityVector, double angularRotation){
        List<Double> angles = new ArrayList<>();
        List<Double> speeds = new ArrayList<>();

        for (int i = 0; i < 2; i++) {
            double vx = velocityVector.x + angularRotation * modulePoses.get(i).y;  // x component
            double vy = velocityVector.y - angularRotation * modulePoses.get(i).x;  // y component

            double angle = Math.atan2(vy,vx);
            double speed = Math.hypot(vx, vy);

            angles.add(angle); // returned in rad
            speeds.add(speed); // returned in in/s

            Log.i("DIFFY DRIVE: ", "GOT ANGLE: " + Math.toDegrees(angle));
            Log.i("DIFFY DRIVE: ", "GOT SPEED: " +speed);
        }


        for(LynxModule hub : allhubs){
            hub.clearBulkCache();
        }

        leftModule.update(speeds.get(0), angles.get(0));
        rightModule.update(speeds.get(1), angles.get(1));
    }
    public Module getLeftModule(){
        return leftModule;
    }
    public Module getRightModule(){
        return rightModule;
    }
}
