package org.firstinspires.ftc.teamcode.Auto;

/**
 * Curvature-aware velocity profile along a pre-sampled path.
 *
 * Per-sample speed cap combines:
 *   1. Global v_max                              (drivetrain top speed)
 *   2. Centripetal limit  √(a_lat / |κ|)          (don't slide out of turns)
 *
 * Then two kinematic sweeps enforce accel/decel. The trick here — the
 * "quadratic acceleration" constraint — is the friction-circle:
 *
 *     a_total = √(a_tangential² + a_centripetal²) ≤ A_max
 *     a_centripetal = v² · κ           (quadratic in v)
 *     ∴ a_tangential_max(v,κ) = √(max(0, A_max² − (v²·κ)²))
 *
 * So the allowed tangential accel shrinks as v grows on a curved segment.
 * The forward/backward sweeps then chain these limits via v² = v₀² + 2·a·Δs.
 *
 * Result: velocity[i] is the fastest v the robot can legally be moving at
 * arc length s[i] given the whole future of the path.
 */
public class MotionProfile {
    public final double[] velocity;   // in/s

    public MotionProfile(BezierSpline path,
                         double maxVel, double maxAccel, double maxDecel,
                         double maxLatAccel,
                         double entrySpeed, double exitSpeed) {
        int n = path.size();
        velocity = new double[n];

        // --- Pass 0: per-sample velocity cap (global + curvature) ---
        for (int i = 0; i < n; i++) {
            double k = Math.abs(path.get(i).curvature);
            double vCap = maxVel;
            if (k > 1e-9) {
                vCap = Math.min(vCap, Math.sqrt(maxLatAccel / k));
            }
            velocity[i] = vCap;
        }

        // --- Forward pass: accel-limited, shrinking tangential budget with v²κ ---
        velocity[0] = Math.min(velocity[0], Math.max(entrySpeed, 0));
        for (int i = 1; i < n; i++) {
            double ds = path.get(i).s - path.get(i - 1).s;
            double v  = velocity[i - 1];
            double k  = Math.abs(path.get(i - 1).curvature);
            double aCent = v * v * k;
            double aTan  = Math.sqrt(Math.max(0, maxAccel * maxAccel - aCent * aCent));
            double vNext = Math.sqrt(v * v + 2 * aTan * ds);
            velocity[i] = Math.min(velocity[i], vNext);
        }

        // --- Backward pass: decel-limited, same friction-circle rule ---
        velocity[n - 1] = Math.min(velocity[n - 1], Math.max(exitSpeed, 0));
        for (int i = n - 2; i >= 0; i--) {
            double ds = path.get(i + 1).s - path.get(i).s;
            double v  = velocity[i + 1];
            double k  = Math.abs(path.get(i + 1).curvature);
            double aCent = v * v * k;
            double aTan  = Math.sqrt(Math.max(0, maxDecel * maxDecel - aCent * aCent));
            double vPrev = Math.sqrt(v * v + 2 * aTan * ds);
            velocity[i] = Math.min(velocity[i], vPrev);
        }
    }

    public double at(int i) { return velocity[i]; }
    public int size()       { return velocity.length; }
}
