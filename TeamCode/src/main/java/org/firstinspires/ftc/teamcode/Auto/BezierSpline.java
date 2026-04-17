package org.firstinspires.ftc.teamcode.Auto;

import java.util.ArrayList;
import java.util.List;

/**
 * C1-continuous piecewise cubic Bézier spline through a list of waypoints.
 *
 * Tangents are estimated Catmull-Rom style:
 *     T_i = tension · (P_{i+1} − P_{i−1})
 * which is then converted into the interior Bézier control points for the
 * segment between P_i and P_{i+1}:
 *     B0 = P_i
 *     B1 = P_i + T_i / 3
 *     B2 = P_{i+1} − T_{i+1} / 3
 *     B3 = P_{i+1}
 *
 * After construction the spline is dense-sampled and each sample carries
 * its arc length, tangent heading, and signed curvature — everything the
 * motion profiler and the follower need without re-evaluating Bernstein
 * polynomials at runtime.
 */
public class BezierSpline {

    public static class Sample {
        public final double s;          // arc length from start (in)
        public final double x, y;       // field-frame position (in)
        public final double tangent;    // path tangent heading (rad)
        public final double curvature;  // signed curvature κ (rad/in)
        public final int segIdx;        // index of the cubic segment

        public Sample(double s, double x, double y, double tangent, double curvature, int segIdx) {
            this.s = s; this.x = x; this.y = y;
            this.tangent = tangent; this.curvature = curvature;
            this.segIdx = segIdx;
        }
    }

    public final List<Pose2d> waypoints;
    public final List<Sample> samples = new ArrayList<>();
    public final double[] waypointS;   // arc length at each original waypoint
    public final double totalLength;

    public BezierSpline(List<Pose2d> waypoints, double tension, int samplesPerSegment) {
        if (waypoints.size() < 2) {
            throw new IllegalArgumentException("BezierSpline needs at least 2 waypoints");
        }
        this.waypoints = new ArrayList<>(waypoints);
        int n = waypoints.size();

        // --- Build cubic Bézier control points for each segment ---
        double[][] seg = new double[n - 1][8]; // [x0,y0,x1,y1,x2,y2,x3,y3]
        for (int i = 0; i < n - 1; i++) {
            Pose2d p0 = waypoints.get(Math.max(i - 1, 0));
            Pose2d p1 = waypoints.get(i);
            Pose2d p2 = waypoints.get(i + 1);
            Pose2d p3 = waypoints.get(Math.min(i + 2, n - 1));

            double tx1 = tension * (p2.x - p0.x);
            double ty1 = tension * (p2.y - p0.y);
            double tx2 = tension * (p3.x - p1.x);
            double ty2 = tension * (p3.y - p1.y);

            seg[i][0] = p1.x;              seg[i][1] = p1.y;
            seg[i][2] = p1.x + tx1 / 3.0;  seg[i][3] = p1.y + ty1 / 3.0;
            seg[i][4] = p2.x - tx2 / 3.0;  seg[i][5] = p2.y - ty2 / 3.0;
            seg[i][6] = p2.x;              seg[i][7] = p2.y;
        }

        // --- Dense sample, accumulating arc length ---
        waypointS = new double[n];
        waypointS[0] = 0;
        double accS = 0;
        double prevX = seg[0][0], prevY = seg[0][1];

        for (int i = 0; i < n - 1; i++) {
            int kStart = (i == 0) ? 0 : 1;  // skip duplicate endpoint between segments
            for (int k = kStart; k <= samplesPerSegment; k++) {
                double t = (double) k / samplesPerSegment;
                double u = 1 - t;
                double[] s = seg[i];

                // B(t)
                double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
                double x = b0 * s[0] + b1 * s[2] + b2 * s[4] + b3 * s[6];
                double y = b0 * s[1] + b1 * s[3] + b2 * s[5] + b3 * s[7];

                // B'(t)
                double dx = 3 * u * u * (s[2] - s[0])
                          + 6 * u * t * (s[4] - s[2])
                          + 3 * t * t * (s[6] - s[4]);
                double dy = 3 * u * u * (s[3] - s[1])
                          + 6 * u * t * (s[5] - s[3])
                          + 3 * t * t * (s[7] - s[5]);

                // B''(t)
                double ddx = 6 * u * (s[4] - 2 * s[2] + s[0])
                           + 6 * t * (s[6] - 2 * s[4] + s[2]);
                double ddy = 6 * u * (s[5] - 2 * s[3] + s[1])
                           + 6 * t * (s[7] - 2 * s[5] + s[3]);

                double speed = Math.hypot(dx, dy);
                double tangent = Math.atan2(dy, dx);
                double kappa = 0;
                if (speed > 1e-9) {
                    kappa = (dx * ddy - dy * ddx) / (speed * speed * speed);
                }

                double ds = Math.hypot(x - prevX, y - prevY);
                accS += ds;
                samples.add(new Sample(accS, x, y, tangent, kappa, i));
                prevX = x; prevY = y;
            }
            waypointS[i + 1] = accS;
        }
        this.totalLength = accS;
    }

    public int size()             { return samples.size(); }
    public Sample get(int i)      { return samples.get(i); }

    /** Binary search: index whose arc length ≥ s. */
    public int indexAtArcLength(double s) {
        if (s <= 0) return 0;
        if (s >= totalLength) return samples.size() - 1;
        int lo = 0, hi = samples.size() - 1;
        while (lo < hi - 1) {
            int m = (lo + hi) >>> 1;
            if (samples.get(m).s < s) lo = m; else hi = m;
        }
        return hi;
    }

    /** Linear interpolation between adjacent samples at arc length s. */
    public Sample interpolateAt(double s) {
        if (s <= 0) return samples.get(0);
        if (s >= totalLength) return samples.get(samples.size() - 1);
        int i = indexAtArcLength(s);
        Sample a = samples.get(i - 1), b = samples.get(i);
        double span = b.s - a.s;
        if (span < 1e-9) return b;
        double t = (s - a.s) / span;
        double dh = Pose2d.normalizeAngle(b.tangent - a.tangent);
        return new Sample(
                s,
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.tangent + t * dh,
                a.curvature + t * (b.curvature - a.curvature),
                a.segIdx
        );
    }

    /** Closest sample to (px, py), searching forward from startIdx only. */
    public int closestIndex(double px, double py, int startIdx) {
        double best = Double.MAX_VALUE;
        int bestI = startIdx;
        for (int i = Math.max(0, startIdx); i < samples.size(); i++) {
            Sample s = samples.get(i);
            double dx = s.x - px, dy = s.y - py;
            double d = dx * dx + dy * dy;
            if (d < best) { best = d; bestI = i; }
        }
        return bestI;
    }
}
