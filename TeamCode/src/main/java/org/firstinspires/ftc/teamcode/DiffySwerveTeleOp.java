package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.DriveTrain;

// Robot-centric teleop test for the diff swerve drive.
//   left stick  X/Y -> translation (in/s)
//   right stick X   -> rotation (rad/s, CW positive to match the DriveTrain convention)
@TeleOp(name = "Diffy Swerve TeleOp", group = "drive")
public class DiffySwerveTeleOp extends LinearOpMode {

    // Scale factor so drivers can cap peak speed while learning. 1.0 = full free speed.
    private static final double TRANSLATION_SCALE = 0.8;
    private static final double ROTATION_SCALE    = 0.7;

    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap);

        telemetry.addLine("Diffy Swerve ready.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Gamepad y axis is inverted (stick up = -1), so negate to make forward positive.
            double stickX = gamepad1.left_stick_x;
            double stickY = -gamepad1.left_stick_y;
            double rot    = gamepad1.right_stick_x;

            double vx = stickX * drive.MAX_INCHES_PER_SEC * TRANSLATION_SCALE;
            double vy = stickY * drive.MAX_INCHES_PER_SEC * TRANSLATION_SCALE;
            double omega = rot  * drive.MAX_RAD_PER_SEC * ROTATION_SCALE;

            drive.update(new Vector2d(vx, vy), omega);

            telemetry.addData("cmd vx (in/s)", "%.1f", vx);
            telemetry.addData("cmd vy (in/s)", "%.1f", vy);
            telemetry.addData("cmd omega (rad/s)", "%.2f", omega);
            telemetry.addData("L heading (deg)", "%.1f",
                    Math.toDegrees(drive.getLeftModule().getHeading()));
            telemetry.addData("R heading (deg)", "%.1f",
                    Math.toDegrees(drive.getRightModule().getHeading()));
            telemetry.update();
        }
    }
}
