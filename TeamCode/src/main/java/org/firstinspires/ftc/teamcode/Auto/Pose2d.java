package org.firstinspires.ftc.teamcode.Auto;

/**
 * Minimal field-frame pose: (x, y) in inches, heading in radians.
 */
public class Pose2d {
    public double x;
    public double y;
    public double heading; // radians, CCW-positive, 0 = facing +x

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d() {
        this(0, 0, 0);
    }

    public double distanceTo(Pose2d other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    /** Wraps any angle into (-pi, pi]. */
    public static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.1f°)", x, y, Math.toDegrees(heading));
    }
}
