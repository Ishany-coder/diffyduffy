package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Drive.DriveTrain;

import java.util.List;

/**
 * Autonomous path follower for the differential-swerve drivetrain.
 *
 * Pipeline on {@link #followPath(List)}:
 *   1. Build a C1-continuous piecewise cubic Bézier spline through the
 *      user's waypoints (Catmull-Rom tangents).
 *   2. Dense-sample it by arc length, carrying (x, y, tangent, κ) per sample.
 *   3. Run a curvature-aware velocity profile (forward/backward sweeps with
 *      the friction-circle constraint — see {@link MotionProfile}).
 *   4. Drive it via adaptive pure pursuit: lookahead scales with commanded
 *      speed, the lookahead point is pulled from the spline directly.
 *   5. Within {@link #PATH_END_THRESH} of the final waypoint, hand off to a
 *      terminal go-to-point controller for a precise finish.
 *
 * Terminal / go-to-point controller is swappable via {@link #USE_PID}:
 *   true  → translational PIDF + heading PID  ("tunable everything" mode)
 *   false → SQuID on both translation and heading (fewer knobs, fast settle)
 *
 * Heading along a path is one of {@link HeadingMode}:
 *   TANGENT   — face along the path tangent
 *   CONSTANT  — hold the last waypoint's heading throughout
 *   LINEAR    — interpolate between waypoint headings by arc length (default)
 *
 * Loop-lag compensation: every update predicts the pose {@link #LOOP_TIME_MS}
 * into the future (current pose + velocity · dt) so that the command we send
 * now is consistent with where the robot will actually be when the command
 * takes effect.
 */
@Config
public class Follower {

    // ==================== TUNING VARIABLES ====================

    /** true → translational PIDF + heading PID. false → SQuID on both. */
    public static boolean USE_PID = true;

    // --- Translational PIDF (used when USE_PID = true) ---
    public static double TRANS_KP = 0.15;
    public static double TRANS_KI = 0.01;
    public static double TRANS_KD = 0.02;
    public static double TRANS_KF = 0.03;

    // --- Heading PID (used when USE_PID = true) ---
    public static double HEAD_KP = 1.5;
    public static double HEAD_KI = 0.005;
    public static double HEAD_KD = 0.10;

    // --- SQuID gains (used when USE_PID = false, and by squidGoToPoint) ---
    // output = Kp · sign(e) · √|e|. Tune so Kp·√(typical error) ≈ desired speed.
    // e.g. for 60 in/s at 100 in: Kp ≈ 60/√100 = 6.
    public static double TRANS_SQUID_KP = 6.0;
    public static double HEAD_SQUID_KP  = 4.0;

    // --- Tolerances ---
    public static double POS_TOLERANCE_IN   = 0.5;
    public static double HEAD_TOLERANCE_RAD = Math.toRadians(2.0);

    // --- Motion constraints (profile & go-to-point) ---
    public static double MAX_VEL       = 60.0;  // in/s — auto-clamped to drivetrain top speed
    public static double MAX_ACCEL     = 40.0;  // in/s² tangential accel budget
    public static double MAX_DECEL     = 60.0;  // in/s² tangential decel budget
    public static double MAX_LAT_ACCEL = 30.0;  // in/s² centripetal cap (tire/carpet friction)
    public static double MIN_SPEED     = 2.0;   // in/s floor so we don't stall outside tolerance

    // --- Pure pursuit ---
    public static double LOOKAHEAD_MIN  = 4.0;   // in
    public static double LOOKAHEAD_MAX  = 18.0;  // in
    public static double LOOKAHEAD_GAIN = 0.35;  // s  (L = min + gain · v_target)
    public static double PATH_END_THRESH = 3.0;  // in, switch to terminal phase

    // --- Spline ---
    public static double SPLINE_TENSION      = 0.5;  // 0 = polyline, 0.5 = Catmull-Rom
    public static int SAMPLES_PER_SEGMENT = 60;   // arc-length sampling density

    // --- Loop-lag compensation ---
    /** One FTC loop period in milliseconds. Pose is projected this far forward. */
    public static double LOOP_TIME_MS = 15.0;

    /** Default heading mode for followPath(). */
    public static HeadingMode DEFAULT_HEADING_MODE = HeadingMode.LINEAR;

    // ==================== PUBLIC TYPES ========================

    public enum HeadingMode { TANGENT, CONSTANT, LINEAR }

    private enum Mode { IDLE, GO_TO_POINT, SQUID_GO_TO_POINT, FOLLOW_PATH }

    // ==================== INTERNALS ===========================

    private final DriveTrain drive;
    private final ThreeWheelOdometry odometry;

    private final PIDFController xPID, yPID, headingPID;
    private final SquIDController transSquid, headingSquid;

    private Mode mode = Mode.IDLE;
    private Pose2d targetPose;

    // Path-following state
    private BezierSpline path;
    private MotionProfile profile;
    private int cursorIdx;
    private HeadingMode headingMode = HeadingMode.LINEAR;

    // Go-to-point profile state
    private double profiledSpeed;

    // External speed cap (setMaxSpeed)
    private double maxSpeed;

    // ==================== CONSTRUCTION =========================

    public Follower(DriveTrain drive, ThreeWheelOdometry odometry) {
        this.drive = drive;
        this.odometry = odometry;
        this.maxSpeed = drive.MAX_INCHES_PER_SEC;

        xPID       = new PIDFController(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        yPID       = new PIDFController(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        headingPID = new PIDFController(HEAD_KP,  HEAD_KI,  HEAD_KD);
        transSquid   = new SquIDController(TRANS_SQUID_KP);
        headingSquid = new SquIDController(HEAD_SQUID_KP);
    }

    // ==================== PUBLIC API ===========================

    /** Go-to-point using the active controller (PID when USE_PID, SQuID otherwise). */
    public void goToPoint(Pose2d target) {
        this.targetPose = target;
        this.mode = USE_PID ? Mode.GO_TO_POINT : Mode.SQUID_GO_TO_POINT;
        this.profiledSpeed = 0;
        resetControllers();
    }

    /** Force SQuID go-to-point, regardless of USE_PID. */
    public void squidGoToPoint(Pose2d target) {
        this.targetPose = target;
        this.mode = Mode.SQUID_GO_TO_POINT;
        resetControllers();
    }

    /** Follow a path using the default heading mode. */
    public void followPath(List<Pose2d> waypoints) {
        followPath(waypoints, DEFAULT_HEADING_MODE);
    }

    /** Follow a path with an explicit heading mode. */
    public void followPath(List<Pose2d> waypoints, HeadingMode hMode) {
        double vCap = Math.min(Math.min(MAX_VEL, maxSpeed), drive.MAX_INCHES_PER_SEC);
        this.path    = new BezierSpline(waypoints, SPLINE_TENSION, SAMPLES_PER_SEGMENT);
        this.profile = new MotionProfile(
                path, vCap, MAX_ACCEL, MAX_DECEL, MAX_LAT_ACCEL,
                /*entrySpeed*/ 0, /*exitSpeed*/ 0);
        this.targetPose  = waypoints.get(waypoints.size() - 1);
        this.cursorIdx   = 0;
        this.headingMode = hMode;
        this.mode        = Mode.FOLLOW_PATH;
        this.profiledSpeed = 0;
        resetControllers();
    }

    /** One control-loop iteration. Must be called every loop. */
    public void update() {
        odometry.update();
        refreshGains();

        Pose2d predicted = predictedPose();

        switch (mode) {
            case GO_TO_POINT:       runPidGoToPoint(predicted);   break;
            case SQUID_GO_TO_POINT: runSquidGoToPoint(predicted); break;
            case FOLLOW_PATH:       runFollowPath(predicted);     break;
            default: drive.update(new Vector2d(0, 0), 0);
        }
    }

    public boolean isBusy() {
        if (mode == Mode.IDLE) return false;
        Pose2d cur = odometry.getPose();
        return cur.distanceTo(targetPose) > POS_TOLERANCE_IN
            || Math.abs(Pose2d.normalizeAngle(targetPose.heading - cur.heading))
                > HEAD_TOLERANCE_RAD;
    }

    public void stop() {
        mode = Mode.IDLE;
        drive.update(new Vector2d(0, 0), 0);
    }

    public Pose2d getCurrentPose()   { return odometry.getPose(); }
    public Pose2d getPredictedPose() { return predictedPose(); }
    public void setMaxSpeed(double inPerSec) { this.maxSpeed = inPerSec; }

    // ==================== FOLLOW_PATH ==========================

    private void runFollowPath(Pose2d current) {
        cursorIdx = path.closestIndex(current.x, current.y, cursorIdx);
        BezierSpline.Sample closest = path.get(cursorIdx);

        // Terminal phase — precise finish via go-to-point
        double straightToEnd = current.distanceTo(targetPose);
        if (straightToEnd < PATH_END_THRESH) {
            mode = USE_PID ? Mode.GO_TO_POINT : Mode.SQUID_GO_TO_POINT;
            if (mode == Mode.GO_TO_POINT) runPidGoToPoint(current);
            else                          runSquidGoToPoint(current);
            return;
        }

        double vTarget = profile.at(cursorIdx);

        // Adaptive lookahead: scales with commanded speed
        double L = clamp(LOOKAHEAD_MIN + LOOKAHEAD_GAIN * vTarget,
                         LOOKAHEAD_MIN, LOOKAHEAD_MAX);
        double sLook = Math.min(closest.s + L, path.totalLength);
        BezierSpline.Sample look = path.interpolateAt(sLook);

        double ex = look.x - current.x;
        double ey = look.y - current.y;
        double eNorm = Math.hypot(ex, ey);

        double vxField = 0, vyField = 0;
        if (eNorm > 1e-6) {
            vxField = ex / eNorm * vTarget;
            vyField = ey / eNorm * vTarget;
        }

        double cos = Math.cos(-current.heading);
        double sin = Math.sin(-current.heading);
        double rvx = vxField * cos - vyField * sin;
        double rvy = vxField * sin + vyField * cos;

        double hTarget = targetHeading(closest);
        double eH = Pose2d.normalizeAngle(hTarget - current.heading);
        double omega = clamp(headingControl(eH),
                             -drive.MAX_RAD_PER_SEC, drive.MAX_RAD_PER_SEC);

        drive.update(new Vector2d(rvx, rvy), omega);
    }

    private double targetHeading(BezierSpline.Sample closest) {
        switch (headingMode) {
            case TANGENT:
                return closest.tangent;
            case CONSTANT:
                return targetPose.heading;
            case LINEAR:
            default: {
                double s = closest.s;
                double[] ws = path.waypointS;
                List<Pose2d> wp = path.waypoints;
                for (int i = 1; i < ws.length; i++) {
                    if (s <= ws[i]) {
                        double span = ws[i] - ws[i - 1];
                        double t = span < 1e-9 ? 0 : (s - ws[i - 1]) / span;
                        double h0 = wp.get(i - 1).heading;
                        double dh = Pose2d.normalizeAngle(wp.get(i).heading - h0);
                        return h0 + t * dh;
                    }
                }
                return targetPose.heading;
            }
        }
    }

    // ==================== PID GO-TO-POINT ======================

    private void runPidGoToPoint(Pose2d current) {
        double ex = targetPose.x - current.x;
        double ey = targetPose.y - current.y;
        double dist = Math.hypot(ex, ey);
        double eH = Pose2d.normalizeAngle(targetPose.heading - current.heading);

        // Trapezoidal speed cap. The decel branch uses v² = 2·a·Δs — v vs d is
        // a parabola, not a line. That "quadratic" shape is what stops smoothly.
        double cap = profileSpeed(dist);

        double cos = Math.cos(-current.heading);
        double sin = Math.sin(-current.heading);
        double rex = ex * cos - ey * sin;
        double rey = ex * sin + ey * cos;

        double vx = 0, vy = 0;
        if (dist > 1e-6) {
            vx = xPID.calculate(rex);
            vy = yPID.calculate(rey);
            double mag = Math.hypot(vx, vy);
            if (mag > cap) {
                double s = cap / mag;
                vx *= s; vy *= s;
            }
        }

        double omega = clamp(headingPID.calculate(eH),
                             -drive.MAX_RAD_PER_SEC, drive.MAX_RAD_PER_SEC);
        drive.update(new Vector2d(vx, vy), omega);
    }

    // ==================== SQuID GO-TO-POINT ====================

    private void runSquidGoToPoint(Pose2d current) {
        double ex = targetPose.x - current.x;
        double ey = targetPose.y - current.y;
        double dist = Math.hypot(ex, ey);
        double eH = Pose2d.normalizeAngle(targetPose.heading - current.heading);

        // SQuID scalar speed command, clamped by drivetrain cap and decel curve.
        // Decel: v ≤ √(2·A·d) — quadratic kinematic stopping curve.
        double speed = Math.abs(transSquid.calculate(dist));
        speed = Math.min(speed, maxSpeed);
        double decelCap = Math.sqrt(2 * MAX_DECEL * Math.max(dist, 0));
        speed = Math.min(speed, decelCap);
        if (dist > POS_TOLERANCE_IN) speed = Math.max(speed, MIN_SPEED);

        double vxField = 0, vyField = 0;
        if (dist > 1e-6) {
            vxField = ex / dist * speed;
            vyField = ey / dist * speed;
        }

        double cos = Math.cos(-current.heading);
        double sin = Math.sin(-current.heading);
        double rvx = vxField * cos - vyField * sin;
        double rvy = vxField * sin + vyField * cos;

        double omega = clamp(headingSquid.calculate(eH),
                             -drive.MAX_RAD_PER_SEC, drive.MAX_RAD_PER_SEC);
        drive.update(new Vector2d(rvx, rvy), omega);
    }

    // ==================== SHARED HELPERS =======================

    private double headingControl(double errorRad) {
        return USE_PID ? headingPID.calculate(errorRad)
                       : headingSquid.calculate(errorRad);
    }

    /** Trapezoidal speed cap for stand-alone go-to-point. Linear accel ramp,
     *  kinematic parabola v = √(2·A·d) on the decel side. */
    private double profileSpeed(double distToTarget) {
        double dt = LOOP_TIME_MS / 1000.0;
        profiledSpeed = Math.min(profiledSpeed + MAX_ACCEL * dt, maxSpeed);
        double decelCap = Math.sqrt(2 * MAX_DECEL * Math.max(distToTarget, 0));
        double speed = Math.min(profiledSpeed, Math.min(decelCap, maxSpeed));
        if (distToTarget > POS_TOLERANCE_IN) speed = Math.max(speed, MIN_SPEED);
        return speed;
    }

    /** Pose projected LOOP_TIME_MS into the future using odometry velocity. */
    private Pose2d predictedPose() {
        double dt = LOOP_TIME_MS / 1000.0;
        Pose2d p = odometry.getPose();
        double px = p.x + odometry.getVelocityX() * dt;
        double py = p.y + odometry.getVelocityY() * dt;
        double ph = Pose2d.normalizeAngle(p.heading + odometry.getVelocityH() * dt);
        return new Pose2d(px, py, ph);
    }

    private void resetControllers() {
        xPID.reset();
        yPID.reset();
        headingPID.reset();
    }

    private void refreshGains() {
        xPID.setCoefficients(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        yPID.setCoefficients(TRANS_KP, TRANS_KI, TRANS_KD, TRANS_KF);
        headingPID.setCoefficients(HEAD_KP, HEAD_KI, HEAD_KD, 0);
        transSquid.setPID(TRANS_SQUID_KP);
        headingSquid.setPID(HEAD_SQUID_KP);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
