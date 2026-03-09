package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.drivetrain.Point;
import org.firstinspires.ftc.teamcode.utils.AngleUtils;

public class PathPoint {
    double x;
    double y;
    double theta;// rad
    double v; //mm/s
    double a; //mm/s^2
    double as = 0; //sideways acceleration (0 for now, add centripetal)
    double omega; //dθ/ds * v (where dθ is heading change, ds is distance)
    double alpha; //angluar accel in terms of distance
    double k; //curvature, or 1/radius;
    double dxdi;
    double dydi;
    double tangentDirection;
    public double distanceToPrev = 0;
    public double distanceAlongPath = 0;
    public double distanceToEnd = 0;

    public PathPoint(double x, double y) {
        this.x = x;
        this.y = y;
        theta = 0;
        k = 0;
    }
    public PathPoint(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        k = 0;

    }

    public PathPoint(double x, double y, double theta, double k) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.k = k;
    }

    public double distance(PathPoint point) {
        return Math.sqrt(Math.pow((x - point.x),2) + Math.pow((y - point.y),2));
    }

    public void setSlopes(PathPoint before, PathPoint after) {
        if (before == null) {
            //get dx/di and dy/di
            dxdi = after.x - x;
            dydi = after.y - y;
        } else if (after == null) {
            //get dx/di and dy/di
            dxdi = x - before.x;
            dydi = y - before.y;
        } else {
            //get dx/di and dy/di
            dxdi = (after.x - before.x) / 2;
            dydi = (after.y - before.y) / 2;
        }
        tangentDirection = Math.atan2(dydi, dxdi);
    }

    public void setCurvature(double k) {
        this.k = k;
    }

    /**
     * Calculate the angular velocity needed to reach nextPoint's heading
     * Based on heading change per distance traveled (rad/mm)
     * omega = dθ/ds * v (where dθ is heading change, ds is distance)
     */
    public double calculateOmega(PathPoint nextPoint) {
        if (nextPoint == null) return 0.0;

        // Calculate heading change (wrapped to [-π, π])
        double dTheta = AngleUtils.normalizeAngle(nextPoint.theta - this.theta);

        // Calculate distance to next point
        double ds = this.distance(nextPoint);
        if (ds < 0.0000000001) return 0.0; // Avoid division by zero

        // Angular velocity coefficient (rad/mm)
        double dThetaPerDs = dTheta / ds;

        // Actual omega will be: omega = dThetaPerDs * v
        // But we store the coefficient, not the final omega
        return dThetaPerDs;
    }
    /**
     * Calculate the angular velocity needed to reach nextPoint's heading
     * Based on anglular velocity change per distance traveled (rad/mm/mm)
     * Must have already calculated omegas
     */
    public double calculateAlpha(PathPoint nextPoint) {
        if (nextPoint == null) return 0.0;

        // Calculate heading change (wrapped to [-π, π])
        double dOmega = nextPoint.omega - this.omega;

        // Calculate distance to next point
        double ds = this.distance(nextPoint);
        if (ds < 0.0000000001) return 0.0; // Avoid division by zero

        // Angular velocity coefficient (rad/mm)
        double dOmegaPerDs = dOmega / ds;

        // Actual omega will be: omega = dThetaPerDs * v
        // But we store the coefficient, not the final omega
        return dOmegaPerDs;
    }

    public void setOmegaCoefficient(double dThetaPerDs) {
        // We'll store the coefficient, and calculate actual omega during following
        // based on current velocity
        this.omega = dThetaPerDs;
    }
    public void setAlphaCoefficient(double dThetaPerDs) {
        // We'll store the coefficient, and calculate actual alpha during following
        // based on current velocity
        this.alpha = dThetaPerDs;
    }

    /**
     * Get actual angular velocity for a given speed
     * omega = (dθ/ds) * v
     */
    public double getOmega(double currentSpeed) {
        return this.omega * currentSpeed;
    }

    /**
     * Calculate actual angular acceleration (rad/s^2)
     * Formula: alpha = (d²θ/ds²) * v² + (dθ/ds) * a
     */
    public double getAlpha(double v, double a) {
        // dOmegaDs is d²θ/ds²
        // omega is dθ/ds
        return (this.alpha * v * v) + (this.omega * a);
    }
}