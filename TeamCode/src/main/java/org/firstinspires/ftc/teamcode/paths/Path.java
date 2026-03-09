package org.firstinspires.ftc.teamcode.paths;

import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.CRUISING_VELOCITY;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.MAX_ANGULAR_ACCELERATION;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.MAX_CENTRIPETAL_ACCELERATION;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.MAX_DECELERATION;

import java.util.ArrayList;

public class Path {
    ArrayList<PathPoint> path;
    public Path() {
        path = new ArrayList<>(); // ← ADD THIS
    }

    public void addPoint(PathPoint pathPoint) {
        path.add(pathPoint);
    }

    public int size() {
        return path.size();
    }

    public PathPoint get(int i) {
        return path.get(i);
    }

    public void computePath() {
        //must be in this order
        calculatePathSlopes();
        calculatePathValues();
        calculateMotionProfile();
    }

    public void calculatePathSlopes() {
        for (int i = 0; i < path.size(); i++) {
            if (i == 0) {
                path.get(i).setSlopes(null, path.get(i + 1));
            } else if (i == path.size() - 1) {
                path.get(i).setSlopes(path.get(i - 1), null);
            } else {
                path.get(i).setSlopes(path.get(i - 1), path.get(i + 1));
            }
        }
    }

    public void calculatePathValues() {
        double totalDist = 0;
        for (int i = 0; i < path.size(); i++) {
            if (i == 0) {
                path.get(i).distanceToPrev = 0;
            } else {
                double d = Math.hypot(
                        path.get(i).x - path.get(i-1).x,
                        path.get(i).y - path.get(i-1).y
                );
                path.get(i).distanceToPrev = d;
                totalDist += d;
            }
            path.get(i).distanceAlongPath = totalDist;
        }

        for (int i = 0; i < path.size(); i++){
            path.get(i).distanceToEnd = totalDist - path.get(i).distanceAlongPath;
        }
    }

    public void calculateMotionProfile() {
        //centripetal limit
        path.get(0).v = 0; // Start at rest
        for (int i = 1; i < path.size(); i++) {
            //check that k != 0
            if (path.get(i).k != 0) {
                path.get(i).v = Math.min(Math.sqrt(MAX_CENTRIPETAL_ACCELERATION / path.get(i).k), CRUISING_VELOCITY);
            } else {
                path.get(i).v = CRUISING_VELOCITY;
            }
        }
        // Calculate angular velocity coefficients BEFORE forward/backward passes
        // This ensures angular constraints are considered in speed limits
        for (int i = 0; i < path.size(); i++) {
            PathPoint current = path.get(i);
            if (i < path.size() - 1) {
                PathPoint next = path.get(i + 1);
                double dThetaPerDs = current.calculateOmega(next);
                current.setOmegaCoefficient(dThetaPerDs);

            } else {
                current.setOmegaCoefficient(0.0);
            }
        }
        for (int i = 0; i < path.size(); i++) {
            PathPoint current = path.get(i);
            if (i < path.size() - 1) {
                PathPoint next = path.get(i + 1);
                double dOmegaPerDs = current.calculateAlpha(next);
                current.setAlphaCoefficient(dOmegaPerDs);

            } else {
                current.setAlphaCoefficient(0.0);
            }
        }

        //Longitudinal accel limit
        path.get(0).v = 0; // Start at rest
        for (int i = 1; i < path.size(); i++) {
            PathPoint prev = path.get(i - 1);
            PathPoint curr = path.get(i);
            // vf^2 = vi^2 + 2ad
            double reachableVel = Math.sqrt(Math.pow(prev.v, 2) + 2 * MAX_ACCELERATION * curr.distanceToPrev);
            curr.v = Math.min(curr.v, reachableVel);
        }

        // 3. Backward Pass (Deceleration)
        path.get(path.size() - 1).v = 0; // End at rest
        for (int i = path.size() - 2; i >= 0; i--) {
            PathPoint next = path.get(i + 1);
            PathPoint curr = path.get(i);
            // vi^2 = vf^2 + 2ad
            double safeVel = Math.sqrt(Math.pow(next.v, 2) + 2 * MAX_DECELERATION * next.distanceToPrev);
            curr.v = Math.min(curr.v, safeVel);
        }

        //pass to calculate accelerations
        for (int i = 0; i < path.size() - 1; i++) {
            PathPoint curr = path.get(i);
            PathPoint next = path.get(i + 1);

            // a = (vf^2 - vi^2) / 2d
            if (next.distanceToPrev > 0) {
                curr.a = (Math.pow(next.v, 2) - Math.pow(curr.v, 2)) / (2 * next.distanceToPrev);
            } else {
                curr.a = 0;
            }
        }
        path.get(path.size() - 1).a = 0; // Last point has 0 accel
    }
}