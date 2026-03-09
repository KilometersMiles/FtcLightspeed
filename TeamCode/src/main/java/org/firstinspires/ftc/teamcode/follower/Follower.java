package org.firstinspires.ftc.teamcode.auto.follower.LQR;

import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.A;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.B;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.BACK_LEFT_POLARITY;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.BACK_RIGHT_POLARITY;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.FRONT_LEFT_POLARITY;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.FRONT_RIGHT_POLARITY;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.HEADING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KA;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KAA;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KF_Q;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KF_R;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KF_INITIAL_P;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KF_INITIAL_X;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KS;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KSA;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KV;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.KVA;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.MAX_SPEED_X;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.MAX_SPEED_Y;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.MOTOR_MAX_RPM;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.POSITION_THRESHOLD;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.Q;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.R;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.VELOCITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.auto.follower.LQR.Constants.ZERO_POWER_DECELERATION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.drivetrain.Point;
import org.firstinspires.ftc.teamcode.utils.AngleUtils;
import org.firstinspires.ftc.teamcode.utils.KalmanFilter;
import org.firstinspires.ftc.teamcode.utils.MarixOperations;
import org.firstinspires.ftc.teamcode.utils.RiccatiSolver;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;


public class Follower {
    private Path path;
    public boolean following = false;

    //For running things concurrently with path
    private Map<Double, Runnable> actions = null;
    private ArrayList<Runnable> updateMethods = null;

    private double x, y, theta;  // Current robot position and heading from Pinpoint Odometry
    private double vx, vy, vw, vTotal;  // Current robot velocity and angluar velocity from Pinpoint Odometry
    private double batteryVoltage;
    private Mecanum robot; //Reference to the drivetrain it is running
    // --- KALMAN FILTER INSTANCES --- for velocity
    private final KalmanFilter velXFilter;
    private final KalmanFilter velYFilter;
    private final KalmanFilter velAngularFilter;

    private int previousIndex = 0;
    private int currentIndex = 0;
    private double percentCompleted = 0; //0 - 1, where 1 is 100%
    private int loopCounter = 0;

    public static double[][] K;

    private List<Point> robotPathHistory = new ArrayList<Point>();

    private ElapsedTime loopTimer = new ElapsedTime();

    public Follower(Path path, Mecanum robot) {
        this.path = path;
        this.robot = robot;
        this.velXFilter = new KalmanFilter(KF_Q, KF_R, KF_INITIAL_P, KF_INITIAL_X);
        this.velYFilter = new KalmanFilter(KF_Q, KF_R, KF_INITIAL_P, KF_INITIAL_X);
        this.velAngularFilter = new KalmanFilter(KF_Q, KF_R, KF_INITIAL_P, KF_INITIAL_X);

        K = RiccatiSolver.calculateLQR(A,B,Q,R);
        //init telemtry
        // --- DEBUG TELEMETRY ---
        // --- DEBUG TELEMETRY ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X", x);
        packet.put("Y", y);
        packet.put("Theta", theta);

        // 1. Progress & Path Tracking
        packet.put("Path Index", 0);
        packet.put("Loop hertz", 0);
        packet.put("Path X", 0);
        packet.put("Path Y", 0);
        packet.put("Path W", 0);
        packet.put("VD", 0);

        // 2. Pose Errors (The most important for tuning K)
        packet.put("Error X (Along)", 0);
        packet.put("Error Y (Cross)", 0);
        packet.put("Error Heading (Deg)", 0);

        // 3. Velocity Errors
        packet.put("Error Vx", 0);
        packet.put("Error Vy", 0);
        packet.put("Error Vt", 0);

        packet.put("LQR Gain K", 0);

        packet.put("Target V", 0);
        packet.put("Target A", 0);
        packet.put("VTotal", vTotal);

        packet.put("Output X", 0);
        packet.put("Output Y", 0);
        packet.put("Output Heading", 0);
        packet.put("Raw VX", 0);
        packet.put("Filtered VX", 0);
        packet.put("Raw VTotal", Math.sqrt(0*0 + 0*0));
        packet.put("Heading velocity", vw);
        packet.put("Filtered VTotal", 0);



        // 6. Visualizing on Dashboard Field
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setStroke("green")
                .strokeCircle(0, 0, 2) // Target point
                .setStroke("red")
                .strokeLine(0, 0, 0 + Math.cos(0)*4, 0 + Math.sin(0)*4); // Robot heading

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }

    public void start() {
        following = true;
        previousIndex = 0;
        currentIndex = 0;
        percentCompleted = 0;
    }

    public void stop() {
        following = false;
        robot.setWheelPowers(0,0, 0, 0);
    }

    public void update() {
        if (following) {
            //update timer
            double currentDt = loopTimer.seconds();
            loopTimer.reset();
            //update localization
            updateLocalization();
            updateBatteryVoltage();
            //track where the robot has been (for graphing)
            robotPathHistory.add(new Point(x,y));
            //find closest point
            currentIndex = getClosestPointIndex();
            PathPoint currentPoint = path.get(currentIndex);
            //update percent completed
            percentCompleted = currentPoint.distanceAlongPath / (currentPoint.distanceAlongPath + currentPoint.distanceToEnd);

            //get feedforward vd,ωd in path frame
            double vd = currentPoint.v;
            double ad = currentPoint.a;
            double sd = 0;
            double asd = currentPoint.as;//calculate centripetal acceleration
            double omega = currentPoint.getOmega(vTotal);
            double alpha = currentPoint.getAlpha(vTotal, ad);

            //zero power decay https://pedropathing.com/docs/pathing/reference/drive-vector-algorithm
            double vf = Math.sqrt(Math.pow(vTotal, 2) + (2 * ZERO_POWER_DECELERATION * currentPoint.distanceToEnd));
            double pd = 0;
            if (vf < vTotal) {
                pd = vTotal - vf;
            }

            vd -= pd;

            // Compute errors in PATH frame (aligned with desired heading)
            double cos_theta_d = Math.cos(currentPoint.tangentDirection);
            double sin_theta_d = Math.sin(currentPoint.tangentDirection);

            // Transform robot position to path frame
            double dx = x - currentPoint.x;
            double dy = y - currentPoint.y;

            double ex = dx * cos_theta_d + dy * sin_theta_d;  // Along-path error
            double ey = -dx * sin_theta_d + dy * cos_theta_d; // Cross-path error
            double et = theta - currentPoint.theta;

            // Velocity errors in path frame
            double v_robot_path = vx * cos_theta_d + vy * sin_theta_d;
            double s_robot_path = -vx * sin_theta_d + vy * cos_theta_d;

            double evx = v_robot_path - vd;
            double evy = s_robot_path - sd;  // Desired lateral velocity is usually 0
            double evt = vw - omega;

            // State vector (6x1)
            double[][] stateError = {{ex}, {ey}, {et}, {evx}, {evy}, {evt}};

            //multiply error matrix by K for output
            double[][] correction = MarixOperations.multiply(K, stateError);

//            double targetV_path = vd - (correction[0][0] * currentDt);
//            double targetS_path = sd - (correction[1][0] * currentDt);
//            double targetW = omega - (correction[2][0] * currentDt);

            //double[] powers = calculateMotorPowers(targetV_path, targetS_path, targetW, currentPoint.tangentDirection);

            double[] tangent = {ad - correction[0][0], vd};
            double[] normal = {asd - correction[1][0], sd};
            double[] rotation = {alpha  - correction[2][0], omega};
            double[] powers = calculateMotorPowers(tangent, normal, rotation, currentPoint.tangentDirection);

            robot.setWheelPowers(powers[0],powers[1], powers[2], powers[3]);

            if (loopCounter++ % 10 == 0) {
                // --- DEBUG TELEMETRY ---
                TelemetryPacket packet = new TelemetryPacket();

                //bot pos
                packet.put("X", x);
                packet.put("Y", y);
                packet.put("Theta", theta);
                // 1. Progress & Path Tracking
                packet.put("Path Index", currentIndex + "/" + (path.size()-1));
                packet.put("Loop hertz", 1/currentDt);
                packet.put("Path X", currentPoint.x);
                packet.put("Path Y", currentPoint.y);
                packet.put("Path W", omega);
                packet.put("VD", vd);

                // 2. Pose Errors (The most important for tuning K)
                packet.put("Error X (Along)", ex);
                packet.put("Error Y (Cross)", ey);
                packet.put("Error Heading (Deg)", Math.toDegrees(et));

                // 3. Velocity Errors
                packet.put("Error Vx", evx);
                packet.put("Error Vy", evy);
                packet.put("Error Vt", evt);

                packet.put("Target V", vd);
                packet.put("Target A", ad);

                packet.put("VTotal", vTotal);
                packet.put("Heading velocity", vw);

                packet.put("Output X", correction[0][0]);
                packet.put("Output Y", correction[1][0]);
                packet.put("Output Heading", correction[2][0]);

                // 6. Visualizing on Dashboard Field
                packet.fieldOverlay()
                        .setStrokeWidth(1)
                        .setStroke("green")
                        .strokeCircle(currentPoint.x/25.4, currentPoint.y/25.4, 2) // Target point
                        .setStroke("yellow")
                        .strokeLine(x/25.4, y/25.4, x/25.4 + Math.cos(-theta)*4, y/25.4 + Math.sin(-theta)*4); // Robot heading

                graphPath(packet);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
        if (isDone()) {
            stop();
        }
    }

//    private double[] calculateMotorPowers(double vTangent, double vNormal, double omega, double tangentDirection) {
//        // --- 1. SETUP & TRANSFORMS ---
//        // We need to calculate the robot-centric contributions of each component separately.
//
//        // Angle from Robot Heading to Path Tangent
//        // (This combines the Path->Field and Field->Robot rotations into one step)
//        double relativeHeading = tangentDirection - theta;
//        double cosRel = Math.cos(relativeHeading);
//        double sinRel = Math.sin(relativeHeading);
//
//        // Transform Path-Centric Velocities to Robot-Centric Velocities
//        // Note: vTangent is along X in path frame, vNormal is along Y in path frame
//
//        // Component 1: Normal (Lateral Correction) - PRIORITY 1
//        // In path frame: (0, vNormal)
//        double norm_rob_x = -vNormal * sinRel;
//        double norm_rob_y = vNormal * cosRel;
//
//        // Component 2: Heading (Omega) - PRIORITY 2
//        // Just omega, applied directly to kinematic equation
//
//        // Component 3: Tangent (Forward/Path Speed) - PRIORITY 3
//        // In path frame: (vTangent, 0)
//        double tan_rob_x = vTangent * cosRel;
//        double tan_rob_y = vTangent * sinRel;
//
//        // Apply Lateral Multiplier (Mecanum drift compensation) to Y components
//        double lateralMultiplier = MAX_SPEED_X / MAX_SPEED_Y;
//        norm_rob_y *= lateralMultiplier;
//        tan_rob_y  *= lateralMultiplier;
//
//        // --- 2. CALCULATE RAW POWERS PER COMPONENT ---
//        // We calculate what "100%" of each component looks like in terms of motor power
//
//        double[] pNormal  = new double[4];
//        double[] pHeading = new double[4];
//        double[] pTangent = new double[4];
//
//        // Constants for conversion
//        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS;
//        double max_rpm = MOTOR_MAX_RPM;
//        // Kinematics factor for rotation: (lx + ly)
//        double rotFactor = Constants.ROBOT_RADIUS;
//
//        // Helper function to convert robot velocity to motor power
//        // V_wheel = Vx - Vy (+/- rot)
//        // Power = (V_wheel / Circ) * 60 / MaxRPM
//
//        // Calculate Normal Powers (fl, bl, fr, br)
//        pNormal[0] = velocityToPower(norm_rob_x - norm_rob_y, wheelCircumference, max_rpm);
//        pNormal[1] = velocityToPower(norm_rob_x + norm_rob_y, wheelCircumference, max_rpm);
//        pNormal[2] = velocityToPower(norm_rob_x + norm_rob_y, wheelCircumference, max_rpm);
//        pNormal[3] = velocityToPower(norm_rob_x - norm_rob_y, wheelCircumference, max_rpm);
//
//        // Calculate Heading Powers
//        double rotVel = omega * rotFactor;
//        double rotPower = velocityToPower(rotVel, wheelCircumference, max_rpm);
//        pHeading[0] = -rotPower; // FL - Note: Check your specific motor directions!
//        pHeading[1] = rotPower; // BL
//        pHeading[2] = -rotPower; // FR
//        pHeading[3] = rotPower; // BR
//
//        // Calculate Tangent Powers
//        pTangent[0] = velocityToPower(tan_rob_x - tan_rob_y, wheelCircumference, max_rpm);
//        pTangent[1] = velocityToPower(tan_rob_x + tan_rob_y, wheelCircumference, max_rpm);
//        pTangent[2] = velocityToPower(tan_rob_x + tan_rob_y, wheelCircumference, max_rpm);
//        pTangent[3] = velocityToPower(tan_rob_x - tan_rob_y, wheelCircumference, max_rpm);
//
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("PNormal", pNormal[0] + ", " + pNormal[1] + ", " + pNormal[2] + ", " + pNormal[3]);
//        packet.put("pHeading", pHeading[0] + ", " + pHeading[1] + ", " + pHeading[2] + ", " + pHeading[3]);
//        packet.put("pTangent", pTangent[0] + ", " + pTangent[1] + ", " + pTangent[2] + ", " + pTangent[3]);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboard.sendTelemetryPacket(packet);
//
//        // --- 3. STACK AND PRIORITIZE ---
//
//        double[] finalPowers = new double[4];
//
//        // A. Apply Priority 1: Normal (Lateral Correction)
//        // If this alone exceeds 1.0, we must scale it down (physics limit), but it gets 100% of available.
//        double maxNorm = 0;
//        for(double p : pNormal) maxNorm = Math.max(maxNorm, Math.abs(p));
//
//        double scaleNorm = (maxNorm > 1.0) ? (1.0 / maxNorm) : 1.0;
//
//        for (int i = 0; i < 4; i++) {
//            finalPowers[i] = pNormal[i] * scaleNorm;
//        }
//
//        // B. Apply Priority 2: Heading
//        // We only use the "headroom" remaining after Normal.
//        double maxHeadroomForHeading = 1.0;
//        // Find the most restricted wheel
//        for (int i = 0; i < 4; i++) {
//            double used = Math.abs(finalPowers[i]);
//            double room = 1.0 - used;
//            // How much of pHeading[i] fits in 'room'?
//            // If pHeading is 0, we have infinite scaling room.
//            if (Math.abs(pHeading[i]) > 1e-6) {
//                double allow = room / Math.abs(pHeading[i]);
//                maxHeadroomForHeading = Math.min(maxHeadroomForHeading, allow);
//            }
//        }
//
//        // Apply scaled heading
//        for (int i = 0; i < 4; i++) {
//            finalPowers[i] += pHeading[i] * maxHeadroomForHeading;
//        }
//
//        // C. Apply Priority 3: Tangent (Speed along path)
//        // Use whatever headroom is left after Normal AND Heading
//        double maxHeadroomForTangent = 1.0;
//        for (int i = 0; i < 4; i++) {
//            double used = Math.abs(finalPowers[i]);
//            double room = 1.0 - used;
//            if (Math.abs(pTangent[i]) > 1e-6) {
//                double allow = room / Math.abs(pTangent[i]);
//                maxHeadroomForTangent = Math.min(maxHeadroomForTangent, allow);
//            }
//        }
//
//        // Apply scaled tangent
//        for (int i = 0; i < 4; i++) {
//            finalPowers[i] += pTangent[i] * maxHeadroomForTangent;
//        }
//
//        // Correct for signs based on your specific motor config
//        // (Original code had: -fl, bl, -fr, br)
//        return new double[] { FRONT_LEFT_POLARITY * finalPowers[0], BACK_LEFT_POLARITY * finalPowers[1], FRONT_RIGHT_POLARITY * finalPowers[2], BACK_RIGHT_POLARITY * finalPowers[3] };
//    }

    //arrays - {acceleration, velocity}
    private double[] calculateMotorPowers(double[] tangent, double[] normal, double[] rotation, double tangentDirection) {
        // --- 1. SETUP & TRANSFORMS ---
        // We need to calculate the robot-centric contributions of each component separately.

        // Angle from Robot Heading to Path Tangent
        // (This combines the Path->Field and Field->Robot rotations into one step)
        double relativeHeading = tangentDirection - theta;
        double cosRel = Math.cos(relativeHeading);
        double sinRel = Math.sin(relativeHeading);

        // Transform Path-Centric Velocities to Robot-Centric Velocities
        // Note: vTangent is along X in path frame, vNormal is along Y in path frame

        // Component 1: Normal (Lateral Correction) - PRIORITY 1
        // In path frame: (0, vNormal)
        double norm_rob_x_v = -normal[1] * sinRel;
        double norm_rob_y_v = normal[1] * cosRel;
        double norm_rob_x_a = -normal[0] * sinRel;
        double norm_rob_y_a = normal[0] * cosRel;

        // Component 2: Heading (Omega) - PRIORITY 2
        // Just omega, applied directly to kinematic equation

        // Component 3: Tangent (Forward/Path Speed) - PRIORITY 3
        // In path frame: (vTangent, 0)
        double tan_rob_x_v = tangent[1] * cosRel;
        double tan_rob_y_v = tangent[1] * sinRel;
        double tan_rob_x_a = tangent[0] * cosRel;
        double tan_rob_y_a = tangent[0] * sinRel;

        // Apply Lateral Multiplier (Mecanum drift compensation) to Y components
        double lateralMultiplier = MAX_SPEED_X / MAX_SPEED_Y;
        norm_rob_y_v *= lateralMultiplier;
        tan_rob_y_v  *= lateralMultiplier;
        norm_rob_y_a *= lateralMultiplier;
        tan_rob_y_a  *= lateralMultiplier;

        // --- 2. CALCULATE RAW POWERS PER COMPONENT ---
        // We calculate what "100%" of each component looks like in terms of motor power

        double[] pNormal  = new double[4];
        double[] pHeading = new double[4];
        double[] pTangent = new double[4];

        // Kinematics factor for rotation: (lx + ly)
        double rotFactor = Constants.ROTATION_FACTOR;

        // Helper function to convert robot velocity to motor power
        // V_wheel = Vx - Vy (+/- rot)
        // Power = (V_wheel / Circ) * 60 / MaxRPM

        // Calculate Normal Powers (fl, bl, fr, br)
        pNormal[0] = AVToPower(norm_rob_x_a - norm_rob_y_a, norm_rob_x_v - norm_rob_y_v);
        pNormal[1] = AVToPower(norm_rob_x_a + norm_rob_y_a, norm_rob_x_v + norm_rob_y_v);
        pNormal[2] = AVToPower(norm_rob_x_a + norm_rob_y_a, norm_rob_x_v + norm_rob_y_v);
        pNormal[3] = AVToPower(norm_rob_x_a - norm_rob_y_a, norm_rob_x_v - norm_rob_y_v);

        // Calculate Heading Powers
        double rotPower = AVToPowerAngular(rotation[0], rotation[1]);
        pHeading[0] = rotPower; // FL - Note: Check your specific motor directions!
        pHeading[1] = -rotPower; // BL
        pHeading[2] = rotPower; // FR
        pHeading[3] = -rotPower; // BR

        // Calculate Tangent Powers
        pTangent[0] = AVToPower(tan_rob_x_a - tan_rob_y_a, tan_rob_x_v - tan_rob_y_v);
        pTangent[1] = AVToPower(tan_rob_x_a + tan_rob_y_a, tan_rob_x_v + tan_rob_y_v);
        pTangent[2] = AVToPower(tan_rob_x_a + tan_rob_y_a, tan_rob_x_v + tan_rob_y_v);
        pTangent[3] = AVToPower(tan_rob_x_a - tan_rob_y_a, tan_rob_x_v - tan_rob_y_v);

        // --- 3. STACK AND PRIORITIZE ---

        double[] finalPowers = new double[4];

        // A. Apply Priority 1: Normal (Lateral Correction)
        // If this alone exceeds 1.0, we must scale it down (physics limit), but it gets 100% of available.
        double maxNorm = 0;
        for(double p : pNormal) maxNorm = Math.max(maxNorm, Math.abs(p));

        double scaleNorm = (maxNorm > 1.0) ? (1.0 / maxNorm) : 1.0;

        for (int i = 0; i < 4; i++) {
            finalPowers[i] = pNormal[i] * scaleNorm;
        }

        // B. Apply Priority 2: Heading
        // We only use the "headroom" remaining after Normal.
        double maxHeadroomForHeading = 1.0;
        // Find the most restricted wheel
        for (int i = 0; i < 4; i++) {
            double used = Math.abs(finalPowers[i]);
            double room = 1.0 - used;
            // How much of pHeading[i] fits in 'room'?
            // If pHeading is 0, we have infinite scaling room.
            if (Math.abs(pHeading[i]) > 1e-6) {
                double allow = room / Math.abs(pHeading[i]);
                maxHeadroomForHeading = Math.min(maxHeadroomForHeading, allow);
            }
        }

        // Apply scaled heading
        for (int i = 0; i < 4; i++) {
            finalPowers[i] += pHeading[i] * maxHeadroomForHeading;
        }

        // C. Apply Priority 3: Tangent (Speed along path)
        // Use whatever headroom is left after Normal AND Heading
        double maxHeadroomForTangent = 1.0;
        for (int i = 0; i < 4; i++) {
            double used = Math.abs(finalPowers[i]);
            double room = 1.0 - used;
            if (Math.abs(pTangent[i]) > 1e-6) {
                double allow = room / Math.abs(pTangent[i]);
                maxHeadroomForTangent = Math.min(maxHeadroomForTangent, allow);
            }
        }

        // Apply scaled tangent
        for (int i = 0; i < 4; i++) {
            finalPowers[i] += pTangent[i] * maxHeadroomForTangent;
        }

        // Correct for signs based on your specific motor config
        // (Original code had: -fl, bl, -fr, br)
        return new double[] { FRONT_LEFT_POLARITY * finalPowers[0], BACK_LEFT_POLARITY * finalPowers[1], FRONT_RIGHT_POLARITY * finalPowers[2], BACK_RIGHT_POLARITY * finalPowers[3] };

    }

    public void followPath() {
        this.start();
        while (robot.opMode.opModeIsActive() && !isDone()) {
            this.update();

            //Check to run events
            if (this.actions != null && !this.actions.isEmpty()) {
                ArrayList<Double> keysToRemove = new ArrayList<>();

                for (Map.Entry<Double, Runnable> entry : this.actions.entrySet()) {
                    if (percentCompleted > entry.getKey()) {
                        entry.getValue().run();
                        keysToRemove.add(entry.getKey()); // Collect keys to remove
                    }
                }

                // Remove all collected keys after iteration, so they do not run again
                for (Double key : keysToRemove) {
                    this.actions.remove(key);
                }
            }

            //Call all update methods
            for (Runnable method : updateMethods) {
                method.run();
            }
        }
        stop();
    }

    private double velocityToPower(double vel, double circumference, double maxRpm) {
        double rpm = (vel / circumference) * 60.0;
        return rpm / maxRpm;
    }

    private double AVToPower(double acceleration, double velocity) {
        double voltageFF = (KA * acceleration) + (KV * velocity) + (KS * Math.signum(velocity));
        //handles 0 velocity, only acceleration
        if (Math.abs(velocity) < 0.1 && Math.abs(acceleration) > 0.1) {
            voltageFF += KS * Math.signum(acceleration);
        }
        //adjust based on battery voltage
        return voltageFF / batteryVoltage;
    }

    private double AVToPowerAngular(double acceleration, double velocity) {
        double voltageFF = (KAA * acceleration) + (KVA * velocity) + (KSA * Math.signum(velocity));
        //handles 0 velocity, only acceleration
        if (Math.abs(velocity) < 0.1 && Math.abs(acceleration) > 0.1) {
            voltageFF += KSA * Math.signum(acceleration);
        }
        //adjust based on battery voltage
        return voltageFF / batteryVoltage;
    }

    private int getClosestPointIndex() {
        int lookAheadLimit = Math.min(previousIndex + 30, path.size()); // Only check 30 points ahead
        int i = previousIndex;
        double shortestDistance = Double.MAX_VALUE;

        for (int j = previousIndex; j < lookAheadLimit; j++) {
            double d = path.get(j).distance(new PathPoint(x,y));
            if (d < shortestDistance) {
                i = j;
                shortestDistance = d;
            }
        }
        previousIndex = i;
        return i;
    }

    public boolean isDone() {
        if (path == null || path.size() == 0) {
            return true; // If there's no path, we are technically "done"
        }
        PathPoint target = path.get(path.size() - 1); // Last point
        return hasArrivedHeading(target.theta) && hasArrivedPoint(target);
    }

    private boolean hasArrivedHeading(double targetHeading) {
        double headingError = Math.toDegrees(Math.abs(targetHeading - theta));
        return headingError < HEADING_THRESHOLD;
    }

    private boolean hasArrivedPoint(PathPoint target) {
        double distance = Math.hypot(target.x - x, target.y - y);
        return distance < POSITION_THRESHOLD && vTotal < VELOCITY_THRESHOLD;
    }

    private void updateLocalization() {
        robot.updateOdo();

        x = robot.odo.getPosX();
        y = robot.odo.getPosY();
        theta = robot.odo.getHeading();

        // Get RAW instantaneous velocities from odometry (these are the noisy measurements)
        vx = robot.odo.getVelX();
        vy = robot.odo.getVelY();
        vw = robot.odo.getHeadingVelocity();

        // --- APPLY KALMAN FILTERING ---
        // 1. Predict (advance the uncertainty)
        velXFilter.predict();
        velYFilter.predict();
        velAngularFilter.predict();

        // 2. Update (fuse the prediction with the noisy measurement)
        velXFilter.update(vx);
        velYFilter.update(vy);
        velAngularFilter.update(vw);

        // 3. Retrieve the filtered (smoothed) values
        double filteredVx = velXFilter.getState();
        double filteredVy = velYFilter.getState();
        double filteredVAngular = velAngularFilter.getState();

        // 4. Update the path follower's state variables with the FILTERED values
        // Note: You must update the variables that the rest of your control loop uses.
        // Assuming vTotal is calculated from these components:
        //vTotalFiltered = Math.sqrt((filteredVx * filteredVx) + (filteredVy * filteredVy));


        //Switching vTotal to raw because its good enough
        vTotal = Math.sqrt((vx * vx) + (vy * vy));
        // If the control loop uses robot.odo.getVelocityX(), you may need to update your ODO class
        // to use these filtered values, or ensure your control loop uses filteredVx/filteredVy
        // directly. For now, we update the local vTotal. If you have separate state variables
        // for filtered velocities, use them here.
    }

    private void updateBatteryVoltage() {
        batteryVoltage = robot.getBatteryVoltage();
    }
    public void graphPath(TelemetryPacket packet) {
        int currentI = getClosestPointIndex();

        Canvas overlay = packet.fieldOverlay();
        //blue represents desired path
        overlay.setStroke("blue")
                .setStrokeWidth(1);

        for (int i = 0; i < path.size() - 1; i++) {
            PathPoint p1 = path.get(i);
            PathPoint p2 = path.get(i + 1);
            //draw line from p1 to p2
            overlay.strokeLine(p1.x / 25.4, p1.y / 25.4, p2.x / 25.4, p2.y / 25.4);
        }

        //red represents the path the robot took
        overlay.setStroke("red")
                .setStrokeWidth(1);

        for (int i = 0; i < robotPathHistory.size() - 1; i++) {
            Point p1 = robotPathHistory.get(i);
            Point p2 = robotPathHistory.get(i + 1);

            overlay.strokeLine(p1.x / 25.4, p1.y / 25.4, p2.x / 25.4, p2.y / 25.4);
        }
    }

    public void addAction(double percent, Runnable action) {
        actions.put(percent, action);
    }

    public void addUpdateMethod(Runnable method) {
        updateMethods.add(method);
    }
}