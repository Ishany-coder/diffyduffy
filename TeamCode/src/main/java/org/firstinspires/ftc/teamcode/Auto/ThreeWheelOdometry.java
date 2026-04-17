package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Three dead-wheel localizer.
 *
 * Layout (robot facing +x):
 *   - LEFT  encoder: parallel to forward axis, on the left side of the robot
 *   - RIGHT encoder: parallel to forward axis, on the right side of the robot
 *   - CENTER encoder: perpendicular (strafe axis), somewhere along the robot centerline
 *
 * Uses mid-angle arc integration which is significantly more accurate than
 * naive Euler integration when the robot is turning.
 *
 * All constants below MUST be tuned on your specific robot. The tuning order is:
 *   1. WHEEL_DIAMETER_IN  — measure or use manufacturer spec
 *   2. TICKS_PER_REV      — encoder datasheet (8192 for REV Through-Bore)
 *   3. LEFT_MULT / RIGHT_MULT / CENTER_MULT — set to -1 if encoder reads backward
 *   4. TRACK_WIDTH_IN     — start with a tape-measure estimate, then run the
 *                           heading tuner (spin the robot 10 full turns and adjust
 *                           until odo reports exactly 3600°)
 *   5. FORWARD_OFFSET_IN  — start with tape-measure, then run the strafe tuner
 *                           (strafe left-right repeatedly and adjust until y stays ~0)
 */
@Config  // exposes static fields to FTC Dashboard for live tuning
public class ThreeWheelOdometry {

    // ===================== TUNING CONSTANTS =====================

    /** Ticks per full revolution of the dead-wheel encoder. */
    public static double TICKS_PER_REV = 8192;  // REV Through-Bore

    /** Dead-wheel diameter in inches (35 mm ≈ 1.378 in, 48 mm ≈ 1.890 in). */
    public static double WHEEL_DIAMETER_IN = 1.37795;  // 35 mm omni wheel

    /** Center-to-center distance between the two parallel (forward) wheels. */
    public static double TRACK_WIDTH_IN = 12.0;  // MUST tune

    /**
     * Signed distance from the robot's center of rotation to the strafe wheel.
     * Positive means the strafe wheel is in front of the center of rotation.
     */
    public static double FORWARD_OFFSET_IN = 4.0;  // MUST tune

    /** Set to -1.0 to reverse any encoder that reads the wrong direction. */
    public static double LEFT_MULT   =  1.0;
    public static double RIGHT_MULT  =  1.0;
    public static double CENTER_MULT =  1.0;

    // ===================== DERIVED CONSTANT =====================

    public static double INCHES_PER_TICK =
            (Math.PI * WHEEL_DIAMETER_IN) / TICKS_PER_REV;

    // ===================== STATE ================================

    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx centerEncoder;

    private int prevLeft, prevRight, prevCenter;

    private final Pose2d pose = new Pose2d();

    // Velocities (computed each update, useful for feedforward)
    private double velocityX = 0;    // in/s, field frame
    private double velocityY = 0;    // in/s, field frame
    private double velocityH = 0;    // rad/s
    private long lastTimeNs;

    /**
     * @param hwMap      FTC hardware map
     * @param leftName   hardware-map name for left parallel encoder
     * @param rightName  hardware-map name for right parallel encoder
     * @param centerName hardware-map name for perpendicular (strafe) encoder
     */
    public ThreeWheelOdometry(HardwareMap hwMap, String leftName, String rightName, String centerName) {
        leftEncoder   = hwMap.get(DcMotorEx.class, leftName);
        rightEncoder  = hwMap.get(DcMotorEx.class, rightName);
        centerEncoder = hwMap.get(DcMotorEx.class, centerName);

        prevLeft   = leftEncoder.getCurrentPosition();
        prevRight  = rightEncoder.getCurrentPosition();
        prevCenter = centerEncoder.getCurrentPosition();
        lastTimeNs = System.nanoTime();
    }

    /**
     * Read encoders and integrate pose.  Call once per control loop.
     */
    public void update() {
        // Recalculate in case Dashboard changed constants at runtime
        INCHES_PER_TICK = (Math.PI * WHEEL_DIAMETER_IN) / TICKS_PER_REV;

        int curLeft   = leftEncoder.getCurrentPosition();
        int curRight  = rightEncoder.getCurrentPosition();
        int curCenter = centerEncoder.getCurrentPosition();

        double dLeft   = (curLeft   - prevLeft)   * INCHES_PER_TICK * LEFT_MULT;
        double dRight  = (curRight  - prevRight)  * INCHES_PER_TICK * RIGHT_MULT;
        double dCenter = (curCenter - prevCenter)  * INCHES_PER_TICK * CENTER_MULT;

        prevLeft   = curLeft;
        prevRight  = curRight;
        prevCenter = curCenter;

        // --- Kinematic equations ---
        // Heading change from the differential of the two parallel wheels
        double dHeading = (dRight - dLeft) / TRACK_WIDTH_IN;

        // Forward displacement (average of parallel wheels)
        double dForward = (dLeft + dRight) / 2.0;

        // Strafe displacement, corrected for the arc the strafe wheel traces when turning
        double dStrafe = dCenter - FORWARD_OFFSET_IN * dHeading;

        // --- Mid-angle integration (much better than Euler when turning) ---
        double avgHeading = pose.heading + dHeading / 2.0;
        double cos = Math.cos(avgHeading);
        double sin = Math.sin(avgHeading);

        double dx = dForward * cos - dStrafe * sin;
        double dy = dForward * sin + dStrafe * cos;

        pose.x += dx;
        pose.y += dy;
        pose.heading += dHeading;
        pose.heading = Pose2d.normalizeAngle(pose.heading);

        // --- Velocity estimation (for feedforward / telemetry) ---
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        if (dt > 0) {
            velocityX = dx / dt;
            velocityY = dy / dt;
            velocityH = dHeading / dt;
        }
        lastTimeNs = now;
    }

    // ===================== GETTERS / SETTERS ====================

    public Pose2d getPose() { return pose; }

    /** Reset the localizer to a known pose (e.g. at the start of auto). */
    public void setPose(Pose2d newPose) {
        pose.x       = newPose.x;
        pose.y       = newPose.y;
        pose.heading = newPose.heading;
    }

    public double getVelocityX() { return velocityX; }
    public double getVelocityY() { return velocityY; }
    public double getVelocityH() { return velocityH; }
}
