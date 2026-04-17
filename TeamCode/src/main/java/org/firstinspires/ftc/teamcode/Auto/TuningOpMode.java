package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * All-in-one tuning OpMode for the 3-dead-wheel odometry and PIDF follower.
 *
 * Select a test by changing {@link #TEST} via FTC Dashboard (or edit the default).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 0 — ENCODER CHECK
 *    Push the robot by hand and verify that all three encoders read
 *    the correct sign and roughly correct distance.
 *    - Push forward  → left & right should increase, center ≈ 0
 *    - Strafe left   → center should increase, left & right ≈ 0
 *    - Spin CCW      → right increases, left decreases
 *    If a sign is wrong, flip the corresponding multiplier in
 *    ThreeWheelOdometry (LEFT_MULT, RIGHT_MULT, CENTER_MULT).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 1 — TRACK WIDTH
 *    Manually spin the robot exactly 10 full rotations (3600°) in place.
 *    The telemetry shows the heading the odometry computed.
 *    Adjust ThreeWheelOdometry.TRACK_WIDTH_IN until it reads 3600°.
 *      Formula: new_width = old_width × (reported° / 3600°)
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 2 — FORWARD OFFSET
 *    Manually strafe the robot left and right repeatedly (stay straight).
 *    If the reported Y drifts, adjust ThreeWheelOdometry.FORWARD_OFFSET_IN.
 *    The goal is for Y to stay near 0 throughout the strafe.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 3 — STRAIGHT LINE (translational PID)
 *    The robot drives 48 inches forward using the follower.
 *    Tune Follower.TRANS_KP/KI/KD/KF until it reaches the target
 *    quickly without oscillating.
 *    Start: increase KP until it reaches the target.  If it oscillates,
 *    add KD.  If it stalls short, add small KF.  Add KI last if needed.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 4 — HEADING (heading PID)
 *    The robot rotates 90° in place.
 *    Tune Follower.HEAD_KP/KI/KD.  Same strategy as above.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 5 — DIAGONAL + ROTATION (combined)
 *    The robot drives to (36, 36) while rotating to 180°.
 *    Verifies that translation and heading PIDs work well together.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 6 — SQUARE PATH (pure pursuit)
 *    The robot follows a 36-inch square path using pure pursuit.
 *    Tune Follower.LOOKAHEAD_MIN / LOOKAHEAD_MAX / LOOKAHEAD_GAIN and check
 *    path-tracking accuracy. Lookahead scales with commanded speed:
 *      L = clamp(LOOKAHEAD_MIN + LOOKAHEAD_GAIN · v_target, MIN, MAX).
 *    Larger L = smoother but cuts corners. Smaller L = tighter but oscillates.
 *    Also tune MAX_LAT_ACCEL (centripetal cap) and SPLINE_TENSION.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TEST 7 — REPEATABILITY
 *    Drives to (48, 0) and back to (0, 0) three times.
 *    Check how far the robot drifts from the start after all 3 cycles.
 *    If it drifts: odo constants need more tuning.
 *    If it oscillates: PID gains need adjustment.
 */
@Config
@Autonomous(name = "Tuning OpMode", group = "tuning")
public class TuningOpMode extends LinearOpMode {

    /** Change this in FTC Dashboard to switch tests without redeploying. */
    public static int TEST = 0;

    /** Distance for straight-line test (inches). */
    public static double STRAIGHT_DIST = 48.0;

    /** Heading for the rotation test (degrees). */
    public static double TURN_ANGLE = 90.0;

    /** Square size for the pure-pursuit test (inches). */
    public static double SQUARE_SIZE = 36.0;

    @Override
    public void runOpMode() {
        Planner planner = new Planner(hardwareMap);
        planner.setStartPose(0, 0, 0);

        telemetry.addData("Selected test", TEST);
        telemetry.addLine(describeTest(TEST));
        telemetry.addLine("Press START to run.");
        telemetry.update();
        waitForStart();

        switch (TEST) {
            case 0: runEncoderCheck(planner);       break;
            case 1: runTrackWidthTuner(planner);    break;
            case 2: runForwardOffsetTuner(planner); break;
            case 3: runStraightLine(planner);       break;
            case 4: runHeadingTuner(planner);       break;
            case 5: runDiagonal(planner);           break;
            case 6: runSquarePath(planner);         break;
            case 7: runRepeatability(planner);      break;
            default:
                telemetry.addLine("Unknown test number: " + TEST);
                telemetry.update();
        }
    }

    // ======================== TEST 0 ========================

    private void runEncoderCheck(Planner planner) {
        telemetry.setMsTransmissionInterval(50);
        while (opModeIsActive()) {
            planner.getOdometry().update();
            Pose2d p = planner.getPose();

            telemetry.addLine("=== ENCODER CHECK ===");
            telemetry.addLine("Push robot by hand and verify signs:");
            telemetry.addLine("  Forward → X increases");
            telemetry.addLine("  Strafe left → Y increases");
            telemetry.addLine("  Spin CCW → heading increases");
            telemetry.addLine("");
            telemetry.addData("X (in)",       "%.3f", p.x);
            telemetry.addData("Y (in)",       "%.3f", p.y);
            telemetry.addData("Heading (deg)","%.2f", Math.toDegrees(p.heading));
            telemetry.update();
        }
    }

    // ======================== TEST 1 ========================

    private void runTrackWidthTuner(Planner planner) {
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Spin robot 10 FULL rotations by hand.");
        telemetry.addLine("Then check heading — should read 3600°.");
        telemetry.update();

        double totalHeading = 0;
        double lastHeading = 0;

        while (opModeIsActive()) {
            planner.getOdometry().update();
            double h = Math.toDegrees(planner.getPose().heading);

            // Accumulate total heading (unwrap)
            double delta = h - lastHeading;
            if (delta > 180)  delta -= 360;
            if (delta < -180) delta += 360;
            totalHeading += delta;
            lastHeading = h;

            telemetry.addLine("=== TRACK WIDTH TUNER ===");
            telemetry.addData("Total heading (deg)", "%.2f", totalHeading);
            telemetry.addData("Current heading (deg)", "%.2f", h);
            telemetry.addLine("");
            telemetry.addData("Current TRACK_WIDTH_IN",
                    "%.4f", ThreeWheelOdometry.TRACK_WIDTH_IN);
            telemetry.addLine("");
            telemetry.addLine("If total != 3600 after 10 spins:");
            telemetry.addLine("  new = old × (total / 3600)");
            double suggested = ThreeWheelOdometry.TRACK_WIDTH_IN
                    * (totalHeading / 3600.0);
            if (Math.abs(totalHeading) > 100) {
                telemetry.addData("Suggested TRACK_WIDTH_IN", "%.4f", suggested);
            }
            telemetry.update();
        }
    }

    // ======================== TEST 2 ========================

    private void runForwardOffsetTuner(Planner planner) {
        telemetry.setMsTransmissionInterval(50);
        while (opModeIsActive()) {
            planner.getOdometry().update();
            Pose2d p = planner.getPose();

            telemetry.addLine("=== FORWARD OFFSET TUNER ===");
            telemetry.addLine("Strafe left/right repeatedly.");
            telemetry.addLine("Y should stay near 0.");
            telemetry.addLine("");
            telemetry.addData("X (in)",       "%.3f", p.x);
            telemetry.addData("Y (in)",       "%.3f", p.y);
            telemetry.addData("Heading (deg)","%.2f", Math.toDegrees(p.heading));
            telemetry.addLine("");
            telemetry.addData("Current FORWARD_OFFSET_IN",
                    "%.4f", ThreeWheelOdometry.FORWARD_OFFSET_IN);
            telemetry.addLine("If Y drifts positive → increase offset");
            telemetry.addLine("If Y drifts negative → decrease offset");
            telemetry.update();
        }
    }

    // ======================== TEST 3 ========================

    private void runStraightLine(Planner planner) {
        planner.moveTo(STRAIGHT_DIST, 0, 0);
        while (opModeIsActive() && planner.isBusy()) {
            planner.update();
            showFollowerTelemetry(planner, "STRAIGHT LINE",
                    STRAIGHT_DIST, 0, 0);
        }
        holdAndDisplay(planner, "STRAIGHT LINE DONE");
    }

    // ======================== TEST 4 ========================

    private void runHeadingTuner(Planner planner) {
        planner.moveTo(0, 0, TURN_ANGLE);
        while (opModeIsActive() && planner.isBusy()) {
            planner.update();
            showFollowerTelemetry(planner, "HEADING TUNER",
                    0, 0, TURN_ANGLE);
        }
        holdAndDisplay(planner, "HEADING DONE");
    }

    // ======================== TEST 5 ========================

    private void runDiagonal(Planner planner) {
        planner.moveTo(36, 36, 180);
        while (opModeIsActive() && planner.isBusy()) {
            planner.update();
            showFollowerTelemetry(planner, "DIAGONAL + ROTATE",
                    36, 36, 180);
        }
        holdAndDisplay(planner, "DIAGONAL DONE");
    }

    // ======================== TEST 6 ========================

    private void runSquarePath(Planner planner) {
        double s = SQUARE_SIZE;
        java.util.List<Pose2d> square = java.util.Arrays.asList(
                new Pose2d(0, 0, 0),
                new Pose2d(s, 0, 0),
                new Pose2d(s, s, 0),
                new Pose2d(0, s, 0),
                new Pose2d(0, 0, 0)
        );
        planner.followPath(square);
        while (opModeIsActive() && planner.isBusy()) {
            planner.update();
            showFollowerTelemetry(planner, "SQUARE (pure pursuit)",
                    0, 0, 0);
        }
        holdAndDisplay(planner, "SQUARE DONE");
    }

    // ======================== TEST 7 ========================

    private void runRepeatability(Planner planner) {
        for (int cycle = 1; cycle <= 3 && opModeIsActive(); cycle++) {
            // Drive out
            planner.moveTo(48, 0, 0);
            while (opModeIsActive() && planner.isBusy()) {
                planner.update();
                Pose2d p = planner.getPose();
                telemetry.addLine("=== REPEATABILITY (cycle " + cycle + "/3) OUT ===");
                telemetry.addData("X", "%.2f", p.x);
                telemetry.addData("Y", "%.2f", p.y);
                telemetry.addData("H", "%.1f°", Math.toDegrees(p.heading));
                telemetry.update();
            }

            // Drive back
            planner.moveTo(0, 0, 0);
            while (opModeIsActive() && planner.isBusy()) {
                planner.update();
                Pose2d p = planner.getPose();
                telemetry.addLine("=== REPEATABILITY (cycle " + cycle + "/3) BACK ===");
                telemetry.addData("X", "%.2f", p.x);
                telemetry.addData("Y", "%.2f", p.y);
                telemetry.addData("H", "%.1f°", Math.toDegrees(p.heading));
                telemetry.update();
            }
        }
        holdAndDisplay(planner, "REPEATABILITY DONE — check drift from (0,0,0)");
    }

    // ======================== HELPERS ========================

    private void showFollowerTelemetry(Planner planner, String label,
                                       double tgtX, double tgtY, double tgtH) {
        Pose2d p = planner.getPose();
        double errX = tgtX - p.x;
        double errY = tgtY - p.y;
        double errH = tgtH - Math.toDegrees(p.heading);

        telemetry.addLine("=== " + label + " ===");
        telemetry.addData("Pose",  "(%.2f, %.2f, %.1f°)", p.x, p.y,
                Math.toDegrees(p.heading));
        telemetry.addData("Target","(%.1f, %.1f, %.1f°)", tgtX, tgtY, tgtH);
        telemetry.addData("Error", "(%.2f, %.2f, %.1f°)", errX, errY, errH);
        telemetry.addData("Dist",  "%.2f in", Math.hypot(errX, errY));
        telemetry.update();
    }

    /**
     * After reaching the target, keep updating (holds position) and show
     * the final error so you can assess steady-state accuracy.
     */
    private void holdAndDisplay(Planner planner, String label) {
        planner.stop();
        while (opModeIsActive()) {
            planner.getOdometry().update();
            Pose2d p = planner.getPose();
            telemetry.addLine("=== " + label + " ===");
            telemetry.addData("Final pose", "(%.3f, %.3f, %.2f°)",
                    p.x, p.y, Math.toDegrees(p.heading));
            telemetry.addLine("Robot is stopped. Verify physical position.");
            telemetry.update();
        }
    }

    private String describeTest(int test) {
        switch (test) {
            case 0: return "Encoder direction check (push by hand)";
            case 1: return "Track width tuner (spin 10 turns by hand)";
            case 2: return "Forward offset tuner (strafe by hand)";
            case 3: return "Straight line — tune TRANS PID";
            case 4: return "Heading — tune HEAD PID";
            case 5: return "Diagonal + rotation — combined test";
            case 6: return "Square path — tune pure pursuit lookahead";
            case 7: return "Repeatability — 3 out-and-back cycles";
            default: return "Unknown";
        }
    }
}
