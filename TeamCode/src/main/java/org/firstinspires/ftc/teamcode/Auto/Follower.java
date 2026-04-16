package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Drive.DriveTrain;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous path follower for the differential-swerve drivetrain.
 *
 * Two modes of operation:
 *
 *   1) GO-TO-POINT — PIDF on each robot-frame axis with trapezoidal motion
 *      profiling.  Best for precise positioning and holding a target pose.
 *
 *   2) PURE PURSUIT — follows a list of waypoints using a lookahead circle.
 *      Produces smoother, faster paths through a sequence of points.  When the
 *      robot is within {@link #PATH_END_THRESHOLD_IN} of the final waypoint the
 *      follower automatically switches to go-to-point for a precise finish.
 *
 * Because this is a swerve drive, translation and heading are fully decoupled:
 * the robot can move in any direction while independently controlling its
 * heading.  The follower exploits this by running separate PID loops for
 * (x, y) translation and heading.
 *
 * Typical usage:
 * <pre>
 *   follower.goToPoint(new Pose2d(24, 0, 0));
 *   while (opModeIsActive() && follower.isBusy()) {
 *       follower.update();
 *   }
 * </pre>
 */
@Config
public class Follower {

    // ==================== TRANSLATIONAL PID ====================
    public static double TRANS_KP = 0.15;
    public static double TRANS_KI = 0.01;
    public static double TRANS_KD = 0.02;
    /** Static feedforward — just enough power to overcome carpet friction. */
    public static double TRANS_KF = 0.03;

    // ==================== HEADING PID ==========================
    public static double HEAD_KP  = 1.5;
    public static double HEAD_KI  = 0.005;
    public static double HEAD_KD  = 0.1;

    // ==================== TOLERANCES ===========================
    /** Position error (inches) below which the follower reports "at target". */
    public static double POS_TOLERANCE_IN   = 0.5;
    /** Heading error (rad) below which the follower reports "at target". */
    public static double HEAD_TOLERANCE_RAD = Math.toRadians(2.0);

    // ==================== MOTION PROFILE =======================
    /** Maximum translational acceleration (in/s²) during the ramp-up phase. */
    public static double MAX_ACCEL  = 40.0;
    /** Maximum translational deceleration (in/s²) near the target. */
    public static double MAX_DECEL  = 60.0;
    /**
     * Minimum creep speed (in/s) so the robot doesn't stall when close to
     * the target but not yet inside tolerance.
     */
    public static double MIN_SPEED  = 2.0;

    // ==================== PURE PURSUIT =========================
    /** Lookahead distance for the circle-line intersection (inches). */
    public static double LOOKAHEAD_IN = 10.0;
    /**
     * When the robot is closer than this to the last waypoint, the follower
     * switches from pure pursuit to go-to-point for a precise finish.
     */
    public static double PATH_END_THRESH = 3.0;

    // ==================== INTERNALS ============================

    private final DriveTrain drive;
    private final ThreeWheelOdometry odometry;

    private final PIDFController xPID;
    private final PIDFController yPID;
    private final PIDFController headingPID;

    private Pose2d targetPose;
    private List<Pose2d> path;
    private int closestIdx;

    private enum Mode { IDLE, GO_TO_POINT, PURE_PURSUIT }
    private Mode mode = Mode.IDLE;

    /** Smoothly-ramped speed so we don't slam from 0 to max. */
    private double profiledSpeed;

    private double maxSpeed;

    // ==================== CONSTRUCTION =========================

    public Follower(DriveTrain drive, ThreeWheelOdometry odometry) {
        this.drive    = drive;
        this.odometry = odometry;
        this.maxSpeed = drive.MAX_INCHES_PER_SEC;

        xPID       = new PIDFController(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        yPID       = new PIDFController(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        headingPID = new PIDFController(HEAD_KP,  HEAD_KI,  HEAD_KD);
    }

    // ==================== PUBLIC API ===========================

    /**
     * Drive to a single target pose using PIDF control + motion profiling.
     * Call {@link #update()} in a loop until {@link #isBusy()} returns false.
     */
    public void goToPoint(Pose2d target) {
        this.targetPose    = target;
        this.mode          = Mode.GO_TO_POINT;
        this.profiledSpeed = 0;
        resetPIDs();
    }

    /**
     * Follow a sequence of waypoints using pure pursuit, finishing with
     * precise go-to-point positioning at the last waypoint.
     */
    public void followPath(List<Pose2d> waypoints) {
        this.path          = new ArrayList<>(waypoints);
        this.targetPose    = waypoints.get(waypoints.size() - 1);
        this.closestIdx    = 0;
        this.mode          = Mode.PURE_PURSUIT;
        this.profiledSpeed = 0;
        resetPIDs();
    }

    /**
     * Run one control-loop iteration.  Reads odometry, computes drive
     * commands, and sends them to the drivetrain.  Must be called every loop.
     */
    public void update() {
        odometry.update();
        Pose2d current = odometry.getPose();

        // Hot-reload PID gains from Dashboard
        xPID.setCoefficients(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        yPID.setCoefficients(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        headingPID.setCoefficients(HEAD_KP, HEAD_KI, HEAD_KD, 0);

        switch (mode) {
            case GO_TO_POINT:
                runGoToPoint(current);
                break;
            case PURE_PURSUIT:
                runPurePursuit(current);
                break;
            default:
                drive.update(new Vector2d(0, 0), 0);
                break;
        }
    }

    /** True while the follower has not yet reached the target within tolerance. */
    public boolean isBusy() {
        if (mode == Mode.IDLE) return false;
        Pose2d cur = odometry.getPose();
        return cur.distanceTo(targetPose) > POS_TOLERANCE_IN
            || Math.abs(Pose2d.normalizeAngle(targetPose.heading - cur.heading))
               > HEAD_TOLERANCE_RAD;
    }

    /** Immediately stop and go idle. */
    public void stop() {
        mode = Mode.IDLE;
        drive.update(new Vector2d(0, 0), 0);
    }

    public Pose2d getCurrentPose() {
        return odometry.getPose();
    }

    /** Cap the translational speed (useful for careful movements). */
    public void setMaxSpeed(double inPerSec) {
        this.maxSpeed = inPerSec;
    }

    // ==================== GO-TO-POINT ==========================

    private void runGoToPoint(Pose2d current) {
        double errFieldX = targetPose.x - current.x;
        double errFieldY = targetPose.y - current.y;
        double dist = Math.hypot(errFieldX, errFieldY);
        double errH = Pose2d.normalizeAngle(targetPose.heading - current.heading);

        // --- Trapezoidal motion profile on translational speed ---
        double desiredSpeed = profileSpeed(dist);

        // --- Rotate field-frame error into robot frame ---
        double cos = Math.cos(-current.heading);
        double sin = Math.sin(-current.heading);
        double robotErrX = errFieldX * cos - errFieldY * sin;
        double robotErrY = errFieldX * sin + errFieldY * cos;

        // --- PIDF on each robot-frame axis ---
        double vx = 0, vy = 0;
        if (dist > 0.1) {
            vx = xPID.calculate(robotErrX);
            vy = yPID.calculate(robotErrY);

            // Clamp resultant speed to the profiled maximum
            double rawSpeed = Math.hypot(vx, vy);
            if (rawSpeed > desiredSpeed) {
                double scale = desiredSpeed / rawSpeed;
                vx *= scale;
                vy *= scale;
            }
        }

        // --- Heading PIDF (output is rad/s) ---
        double omega = headingPID.calculate(errH);
        omega = clamp(omega, -drive.MAX_RAD_PER_SEC, drive.MAX_RAD_PER_SEC);

        drive.update(new Vector2d(vx, vy), omega);
    }

    // ==================== PURE PURSUIT =========================

    private void runPurePursuit(Pose2d current) {
        advanceClosestIndex(current);

        // Near the end of the path? Switch to precise positioning.
        double distEnd = current.distanceTo(targetPose);
        if (distEnd < PATH_END_THRESH) {
            mode = Mode.GO_TO_POINT;
            runGoToPoint(current);
            return;
        }

        Pose2d lookahead = findLookahead(current);

        double errX = lookahead.x - current.x;
        double errY = lookahead.y - current.y;
        double dist = Math.hypot(errX, errY);
        double errH = Pose2d.normalizeAngle(lookahead.heading - current.heading);

        // Rotate to robot frame
        double cos = Math.cos(-current.heading);
        double sin = Math.sin(-current.heading);
        double rx  = errX * cos - errY * sin;
        double ry  = errX * sin + errY * cos;

        // Profile speed based on distance to final target (slow near the end)
        double speed = profileSpeed(distEnd);

        if (dist > 0.1) {
            rx = rx / dist * speed;
            ry = ry / dist * speed;
        }

        double omega = headingPID.calculate(errH);
        omega = clamp(omega, -drive.MAX_RAD_PER_SEC, drive.MAX_RAD_PER_SEC);

        drive.update(new Vector2d(rx, ry), omega);
    }

    /** Walk the closest-point index forward (never backward) along the path. */
    private void advanceClosestIndex(Pose2d current) {
        double best = Double.MAX_VALUE;
        for (int i = closestIdx; i < path.size(); i++) {
            double d = current.distanceTo(path.get(i));
            if (d < best) {
                best = d;
                closestIdx = i;
            }
        }
    }

    /**
     * Standard pure-pursuit lookahead: walk segments from the closest point
     * and return the first intersection of a circle (radius = LOOKAHEAD_IN)
     * centered on the robot with the path.  Falls back to the last waypoint.
     */
    private Pose2d findLookahead(Pose2d current) {
        for (int i = closestIdx; i < path.size() - 1; i++) {
            Pose2d hit = segCircleIntersection(
                    current, path.get(i), path.get(i + 1), LOOKAHEAD_IN);
            if (hit != null) return hit;
        }
        return path.get(path.size() - 1);
    }

    /**
     * Line-segment / circle intersection.  Returns the intersection closest
     * to {@code segEnd} (i.e. furthest along the path), or null.
     */
    private static Pose2d segCircleIntersection(
            Pose2d center, Pose2d segStart, Pose2d segEnd, double r) {
        double dx = segEnd.x - segStart.x;
        double dy = segEnd.y - segStart.y;
        double fx = segStart.x - center.x;
        double fy = segStart.y - center.y;

        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - r * r;
        double disc = b * b - 4 * a * c;

        if (disc < 0) return null;

        double sq = Math.sqrt(disc);
        double t1 = (-b - sq) / (2 * a);
        double t2 = (-b + sq) / (2 * a);

        // Pick the furthest-along intersection that lies on the segment [0,1]
        double t = -1;
        if (t2 >= 0 && t2 <= 1) t = t2;
        else if (t1 >= 0 && t1 <= 1) t = t1;
        if (t < 0) return null;

        double ix = segStart.x + t * dx;
        double iy = segStart.y + t * dy;
        // Linearly interpolate heading between the two waypoints
        double ih = segStart.heading
                  + t * Pose2d.normalizeAngle(segEnd.heading - segStart.heading);
        return new Pose2d(ix, iy, ih);
    }

    // ==================== MOTION PROFILING =====================

    /**
     * Trapezoidal-style speed profile.
     *   - Ramps up from 0 at {@link #MAX_ACCEL}.
     *   - Ramps down as sqrt(2 * MAX_DECEL * distance) near the target.
     *   - Never drops below {@link #MIN_SPEED} so the robot doesn't stall
     *     outside the tolerance zone.
     */
    private double profileSpeed(double distToTarget) {
        // Deceleration curve: the max speed we could be going and still stop in time
        double decelLimit = Math.sqrt(2 * MAX_DECEL * distToTarget);

        // Acceleration ramp (assume ~20 ms loop time)
        double dt = 0.020;
        profiledSpeed = Math.min(profiledSpeed + MAX_ACCEL * dt, maxSpeed);

        double speed = Math.min(profiledSpeed, Math.min(decelLimit, maxSpeed));

        // Floor so we don't stall while still outside tolerance
        if (distToTarget > POS_TOLERANCE_IN) {
            speed = Math.max(speed, MIN_SPEED);
        }
        return speed;
    }

    // ==================== HELPERS ==============================

    private void resetPIDs() {
        xPID.reset();
        yPID.reset();
        headingPID.reset();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
