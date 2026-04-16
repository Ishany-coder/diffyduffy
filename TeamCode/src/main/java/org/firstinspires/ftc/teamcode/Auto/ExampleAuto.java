package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

/**
 * Example autonomous that demonstrates:
 *   1. Single-point go-to with heading control
 *   2. Chained waypoints
 *   3. Pure-pursuit path following
 *   4. Speed limiting for careful movements
 *
 * All coordinates are in inches (field frame), headings in degrees.
 */
@Autonomous(name = "Example Auto", group = "auto")
public class ExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // ---- SETUP ----
        Planner planner = new Planner(hardwareMap);

        // Set the starting pose on the field.
        // (0, 0, 0°) means: at the origin, facing the +x direction.
        // Adjust this to match your actual starting tile/orientation.
        planner.setStartPose(0, 0, 0);

        telemetry.addLine("Example Auto initialized.");
        telemetry.addLine("3-wheel odo + PIDF follower ready.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // ======================================================
            // STEP 1 — Drive 24 inches forward (straight line, same heading)
            // ======================================================
            planner.moveTo(24, 0, 0);
            runUntilDone(planner);

            // ======================================================
            // STEP 2 — Strafe right 24 in while rotating to 90°
            //          (swerve can do both at once!)
            // ======================================================
            planner.moveTo(24, -24, 90);
            runUntilDone(planner);

            // ======================================================
            // STEP 3 — Slow, careful approach (e.g. placing a sample)
            // ======================================================
            planner.setMaxSpeed(10); // cap at 10 in/s
            planner.moveTo(30, -24, 90);
            runUntilDone(planner);
            planner.setMaxSpeed(Double.MAX_VALUE); // restore full speed

            // ======================================================
            // STEP 4 — Pure pursuit path (smooth curve back to origin)
            //          The follower will auto-switch to go-to-point
            //          near the last waypoint for a precise finish.
            // ======================================================
            planner.followPath(Arrays.asList(
                    new Pose2d(30, -24, Math.toRadians(90)),   // current position
                    new Pose2d(36, -12, Math.toRadians(45)),   // curve out
                    new Pose2d(24, 0, Math.toRadians(0)),    // passing through
                    new Pose2d(12, 0, Math.toRadians(0)),    // approach origin
                    new Pose2d(0, 0, Math.toRadians(0))    // finish at origin
            ));
            runUntilDone(planner);

            // Done — hold position until OpMode ends
            planner.stop();
            telemetry.addLine("Auto complete.");
            telemetry.update();
        }
    }

    /**
     * Blocks until the planner finishes or the OpMode is stopped.
     * Continuously updates the control loop and displays telemetry.
     */
    private void runUntilDone(Planner planner) {
        while (opModeIsActive() && planner.isBusy()) {
            planner.update();
            showTelemetry(planner);
        }
    }

    private void showTelemetry(Planner planner) {
        Pose2d p = planner.getPose();
        telemetry.addData("X (in)",       "%.2f", p.x);
        telemetry.addData("Y (in)",       "%.2f", p.y);
        telemetry.addData("Heading (deg)","%.1f", Math.toDegrees(p.heading));
        telemetry.addData("Busy",         planner.isBusy());
        telemetry.update();
    }
}
