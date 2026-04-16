package org.firstinspires.ftc.teamcode.Auto;

/**
 * Reusable PIDF controller with:
 *  - Anti-windup integral clamping
 *  - Integral reset on zero-crossing (prevents overshoot)
 *  - Static feedforward term (overcomes friction/gravity)
 */
public class PIDFController {
    private double kP, kI, kD, kF;

    private double integralSum = 0;
    private double lastError = 0;
    private double maxIntegral = 1.0;
    private boolean hasRun = false;
    private long lastTimeNs;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public PIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    /**
     * Compute the controller output from a pre-computed error.
     * Use this when you need to transform the error before passing it in
     * (e.g. rotating field-frame error into robot frame).
     */
    public double calculate(double error) {
        long now = System.nanoTime();

        if (!hasRun) {
            hasRun = true;
            lastTimeNs = now;
            lastError = error;
            // First iteration: P + F only (no D spike, no I yet)
            return kP * error + kF * Math.signum(error);
        }

        double dt = (now - lastTimeNs) / 1e9;
        if (dt <= 0) return kP * error;
        lastTimeNs = now;

        // --- P ---
        double p = kP * error;

        // --- I with anti-windup ---
        integralSum += error * dt;
        integralSum = clamp(integralSum, -maxIntegral, maxIntegral);
        // Reset integral when error crosses zero to reduce overshoot
        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            integralSum = 0;
        }
        double i = kI * integralSum;

        // --- D ---
        double d = kD * (error - lastError) / dt;

        // --- F (static feedforward, overcomes static friction) ---
        double f = kF * Math.signum(error);

        lastError = error;
        return p + i + d + f;
    }

    /**
     * Convenience: compute error internally as (target - measurement).
     */
    public double calculate(double target, double measurement) {
        return calculate(target - measurement);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        hasRun = false;
    }

    public void setMaxIntegral(double max) { this.maxIntegral = max; }

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getLastError() { return lastError; }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
