package org.firstinspires.ftc.teamcode.auto.follower.LQR;

import org.firstinspires.ftc.teamcode.drivetrain.Point;

import java.util.ArrayList;
import java.util.List;

public class SplineGenerator {
    private List<Point> origWaypoints;
    int numSegments = 100; // The resolution of each spline segment
    double tension = 0.5; // Adjust this value as needed for the curve tightness

    public Path createPath(List<Point> waypoints, double startHeading, double endHeading, HeadingMode mode) {
        double[] xP = new double[waypoints.size()];
        double[] yP = new double[waypoints.size()];
        int i = 0;
        for (Point p : waypoints) {
            xP[i] = waypoints.get(i).x;
            yP[i] = waypoints.get(i).y;
            i++;
        }
        this.origWaypoints = waypoints;
        List<PathPoint> pathWaypoints = generateSplinePath(xP, yP);

        //for each point, determine a heading, add point to path
        Path path = new Path();
        for (int j = 0; j < pathWaypoints.size(); j++) {
            path.addPoint(pathWaypoints.get(j));
        }
        path.calculatePathSlopes();
        switch(mode) {
            case TANGENT:
                // Copy the calculated tangent into the target theta
                for (int j = 0; j < path.size(); j++) {
                    path.get(j).theta = path.get(j).tangentDirection;
                }
                break;
            case CONSTANT:
                for (int j = 0; j < path.size(); j++) {
                    path.get(j).theta = startHeading; // Or interpolate
                }
                break;
            case INTERPOLATED:
                // Interpolate heading from start to end
                for (int j = 0; j < path.size(); j++) {
                    double t = (double)j / (path.size() - 1);
                    path.get(j).theta = startHeading + (t * (endHeading - startHeading));
                }
                break;
        }

        path.computePath();
        return path;
    }

    private List<PathPoint> generateSplinePath(double[] xPoints, double[] yPoints) {
        List<PathPoint> path = new ArrayList<>();
        if (xPoints.length < 2) {
            // Extract the start and end points
            double x1 = xPoints[0];
            double y1 = yPoints[0];
            double x2 = xPoints[1];
            double y2 = yPoints[1];

            // Generate 100 points along the line
            for (int i = 0; i < 100; i++) {
                // Interpolating between the two points
                double t = i / 99.0;  // t varies from 0 to 1
                double x = (double) (x1 + t * (x2 - x1));
                double y = (double) (y1 + t * (y2 - y1));
                PathPoint point = new PathPoint(x, y);
                point.setCurvature(0);
                path.add(point);
            }

            return path; // Not enough points to create a spline
        }

        for (int i = 0; i < xPoints.length - 1; i++) {

            Point p0 = i > 0 ? new Point((int) xPoints[i - 1], (int) yPoints[i - 1]) : new Point((int) xPoints[i], (int) yPoints[i]);
            Point p1 = new Point((int) xPoints[i], (int) yPoints[i]);
            Point p2 = new Point((int) xPoints[i + 1], (int) yPoints[i + 1]);
            Point p3 = i < xPoints.length - 2 ? new Point((int) xPoints[i + 2], (int) yPoints[i + 2]) : p2;


            for (int j = 0; j < numSegments; j++) {
                double t0 = (double) j / numSegments;
                Point interpolatedPoint = catmullRomInterpolate(p0, p1, p2, p3, t0, tension);
                double k = calculateCurvature(p0, p1, p2, p3, t0, tension);

                PathPoint pathPoint = new PathPoint(interpolatedPoint.x, interpolatedPoint.y);
                pathPoint.setCurvature(k); // Assuming you add this setter

                // Add each interpolated point to the path list
                path.add(pathPoint);
            }
        }
        path.add(new PathPoint(xPoints[xPoints.length -1], yPoints[yPoints.length -1]));
        return path;
    }

    private Point catmullRomInterpolate(Point p0, Point p1, Point p2, Point p3, double t, double tension) {
        double t2 = t * t;
        double t3 = t2 * t;
        double a0 = -tension * t3 + 2 * tension * t2 - tension * t;
        double a1 = (2 - tension) * t3 + (tension - 3) * t2 + 1;
        double a2 = (tension - 2) * t3 + (3 - 2 * tension) * t2 + tension * t;
        double a3 = tension * t3 - tension * t2;


        double x = a0 * p0.x + a1 * p1.x + a2 * p2.x + a3 * p3.x;
        double y = a0 * p0.y + a1 * p1.y + a2 * p2.y + a3 * p3.y;
        return new Point(x, y);
    }

    private double calculateCurvature(Point p0, Point p1, Point p2, Point p3, double t, double tension) {
        double t2 = t * t;

        // First Derivatives (Velocity)
        double da0 = -3 * tension * t2 + 4 * tension * t - tension;
        double da1 = 3 * (2 - tension) * t2 + 2 * (tension - 3) * t;
        double da2 = 3 * (tension - 2) * t2 + 2 * (3 - 2 * tension) * t + tension;
        double da3 = 3 * tension * t2 - 2 * tension * t;

        double dx = da0 * p0.x + da1 * p1.x + da2 * p2.x + da3 * p3.x;
        double dy = da0 * p0.y + da1 * p1.y + da2 * p2.y + da3 * p3.y;

        // Second Derivatives (Acceleration)
        double dda0 = -6 * tension * t + 4 * tension;
        double dda1 = 6 * (2 - tension) * t + 2 * (tension - 3);
        double dda2 = 6 * (tension - 2) * t + 2 * (3 - 2 * tension);
        double dda3 = 6 * tension * t - 2 * tension;

        double ddx = dda0 * p0.x + dda1 * p1.x + dda2 * p2.x + dda3 * p3.x;
        double ddy = dda0 * p0.y + dda1 * p1.y + dda2 * p2.y + dda3 * p3.y;

        // Curvature Formula: k = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
        double numerator = Math.abs(dx * ddy - dy * ddx);
        double denominator = Math.pow(dx * dx + dy * dy, 1.5);

        // Handle straight lines (denominator will be 0)
        if (denominator < 1e-9) return 0;

        return numerator / denominator;
    }
}