package org.firstinspires.ftc.teamcode.Auto;

/**
 * Square-root P controller ("SQuID").
 *
 *     output = Kp · sign(error) · √|error|
 *
 * Shape, compared to a linear P term:
 *   - Aggressive near the setpoint — sqrt derivative is infinite at 0, so the
 *     controller keeps pushing even at tiny errors → fast settle.
 *   - Gentle far from the setpoint — the √ curve is sub-linear for |e| > 1,
 *     so it doesn't slam into a command saturation and cause windup.
 *
 * Good default for terminal-phase positioning and for heading control when
 * a full PIDF is overkill. Has no I or D term; overshoot is bounded by the
 * sub-linear far-field response, not by derivative damping.
 */
public class SquIDController {
    private double kp;

    public SquIDController() { this(1.0); }

    public SquIDController(double kp) { this.kp = kp; }

    public void setPID(double kp) { this.kp = kp; }
    public double getKp() { return kp; }

    /** Compute output from (setpoint − measurement). */
    public double calculate(double measurement, double setpoint) {
        return calculate(setpoint - measurement);
    }

    /** Compute output directly from a signed error. */
    public double calculate(double error) {
        return kp * Math.signum(error) * Math.sqrt(Math.abs(error));
    }
}
