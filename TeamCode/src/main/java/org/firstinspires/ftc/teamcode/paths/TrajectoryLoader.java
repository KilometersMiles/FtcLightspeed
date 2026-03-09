package org.firstinspires.ftc.teamcode.paths;

import android.content.res.AssetManager;
import org.json.JSONArray;
import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.IOException;

public class TrajectoryLoader {

    /**
     * Loads a trajectory from the app's assets folder.
     * @param assetManager Pass 'hardwareMap.appContext.getAssets()' from your OpMode.
     * @param fileName The name of the file in the assets folder (e.g., "trajectory.json").
     */
    public static Path loadFromAssets(AssetManager assetManager, String fileName, boolean isBlueAlliance) {
        Path path = new Path();
        StringBuilder jsonString = new StringBuilder();

        // API 24 compatible reading from Assets
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(assetManager.open(fileName)))) {
            String line;
            while ((line = reader.readLine()) != null) {
                jsonString.append(line);
            }

            //parse string to data
            JSONArray jsonArray = new JSONArray(jsonString.toString());
            for (int i = 0; i < jsonArray.length(); i++) {
                JSONObject point = jsonArray.getJSONObject(i);
                double x_mm = point.getDouble("x") * 1000.0;
                double y_mm = point.getDouble("y") * 1000.0;
                double theta = point.getDouble("theta");

                double vx = point.getDouble("vx");
                double vy = point.getDouble("vy");
                double omega = point.getDouble("omega");
                //reflect across x axis if blue
                if (isBlueAlliance) {
                    y_mm = -y_mm;
                    theta = -theta;
                    vy = -vy;
                    omega = -omega;
                }

                PathPoint p = new PathPoint(x_mm, y_mm, theta);

                p.v = Math.hypot(vx, vy) * 1000.0;

                if (p.v > 1e-6) {
                    // Python omega (rad/s) to dTheta/ds coefficient
                    p.setOmegaCoefficient(omega / (p.v));
                } else {
                    p.setOmegaCoefficient(0);
                }

                path.addPoint(p);
            }

            finalizePath(path);

        } catch (IOException e) {
            throw new RuntimeException("Failed to find or open trajectory file: " + fileName, e);
        } catch (Exception e) {
            throw new RuntimeException("Failed to parse JSON for trajectory file: " + fileName, e);
        }

        return path;
    }

    private static void finalizePath(Path path) {
        path.calculatePathSlopes(); //
        path.calculatePathValues(); //

        for (int i = 0; i < path.size() - 1; i++) {
            PathPoint curr = path.get(i);
            PathPoint next = path.get(i + 1);
            double ds = next.distanceToPrev; //

            if (ds > 1e-6) {
                // Linear Acceleration: a = (vf^2 - vi^2) / 2d
                curr.a = (Math.pow(next.v, 2) - Math.pow(curr.v, 2)) / (2 * ds);

                // Angular Acceleration Coefficient: dOmega/ds
                double dOmegaDs = (next.omega - curr.omega) / ds;
                curr.setAlphaCoefficient(dOmegaDs);
            } else {
                curr.a = 0;
                curr.setAlphaCoefficient(0);
            }
        }
    }
}