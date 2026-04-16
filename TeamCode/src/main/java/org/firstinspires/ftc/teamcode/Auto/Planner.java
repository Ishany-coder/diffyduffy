package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drive.DriveTrain;

import java.util.Arrays;
import java.util.List;

/**
 * High-level autonomous planner.  Wraps the Follower so OpModes can queue
 * up moves without managing the control loop themselves.
 *
 * Usage:
 * <pre>
 *   planner.moveTo(24, 0, 0);          // drive 24 in forward
 *   planner.moveTo(24, 24, 90);        // strafe + rotate
 *   planner.followPath(waypoints);      // pure pursuit
 *   // in your loop:
 *   while (opModeIsActive() && planner.isBusy()) planner.update();
 * </pre>
 */
public class Planner {
    private final DriveTrain drive;
    private final ThreeWheelOdometry odometry;
    private final Follower follower;

    /**
     * @param hwMap       FTC hardware map
     * @param leftEnc     hardware-map name for the left dead-wheel encoder
     * @param rightEnc    hardware-map name for the right dead-wheel encoder
     * @param centerEnc   hardware-map name for the center (strafe) dead-wheel encoder
     */
    public Planner(HardwareMap hwMap, String leftEnc, String rightEnc, String centerEnc) {
        drive    = new DriveTrain(hwMap);
        odometry = new ThreeWheelOdometry(hwMap, leftEnc, rightEnc, centerEnc);
        follower = new Follower(drive, odometry);
    }

    /** Convenience: uses default encoder hardware-map names. */
    public Planner(HardwareMap hwMap) {
        this(hwMap, "leftEncoder", "rightEncoder", "centerEncoder");
    }

    /** Set the starting field pose (call before waitForStart). */
    public void setStartPose(double xIn, double yIn, double headingDeg) {
        odometry.setPose(new Pose2d(xIn, yIn, Math.toRadians(headingDeg)));
    }

    /** PID-controlled move to a single target pose. Heading in degrees. */
    public void moveTo(double xIn, double yIn, double headingDeg) {
        follower.goToPoint(new Pose2d(xIn, yIn, Math.toRadians(headingDeg)));
    }

    /** Pure-pursuit path follow through a list of waypoints. */
    public void followPath(List<Pose2d> waypoints) {
        follower.followPath(waypoints);
    }

    /** Must be called every loop iteration. */
    public void update() {
        follower.update();
    }

    /** True while the robot hasn't reached its target yet. */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /** Emergency stop. */
    public void stop() {
        follower.stop();
    }

    /** Cap speed for delicate movements (in/s). */
    public void setMaxSpeed(double inPerSec) {
        follower.setMaxSpeed(inPerSec);
    }

    public Pose2d getPose() {
        return follower.getCurrentPose();
    }

    public Follower getFollower() {
        return follower;
    }

    public ThreeWheelOdometry getOdometry() {
        return odometry;
    }
}
