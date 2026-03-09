package org.firstinspires.ftc.teamcode.auto.follower.LQR;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.RiccatiSolver;

@Config
public class Constants {
    //KALMAN FILTER
    public static double KF_Q = 1;   // Typical range: 0.001 to 0.1
    public static double KF_R = 50.0; // Typical range: 10.0 to 1000.0 (Higher for more outlier rejection)
    public static double KF_INITIAL_P = 100.0;
    public static double KF_INITIAL_X = 0.0;

    //THRESHOLDS
    public static double POSITION_THRESHOLD = 150; // mm tolerance to reach each waypoint
    public static double HEADING_THRESHOLD = 10; // degrees tolerance
    public static double VELOCITY_THRESHOLD = 100; // mm/s tolerance to reach each waypoint

    //KINEMATIC CONSTANTS FOR PATH GENERATION
    public static double MAX_ACCELERATION = 2300.00;// mm/s^2
    public static double MAX_DECELERATION = 1400;// mm/s^2
    public static double CRUISING_VELOCITY = 2000;// mm/s
    public static double MAX_CENTRIPETAL_ACCELERATION = 2000;//mm/s^2
    public static double ZERO_POWER_DECELERATION = 200.0;//mm/s^2
    public static double MAX_ANGULAR_ACCELERATION = 6; // rad/s²
    public static double MAX_ANGULAR_VELOCITY = 6; // rad/s
    public static double MAX_SPEED_X = 1900;//mm/s
    public static double MAX_SPEED_Y = 1680;//mm/s

    //FEEDFORWARD TERMS
    public static double KA = 0.001;// Volts / mm/s^2 or Volts per acceleration
    public static double KV = 0.006;// Volts / mm/s or Volts per velocity
    public static double KS = 0.5058165330173525;// Volts (required to start moving)
    public static double KAA = 0.1;// Volts / rad/s^2 or Volts per acceleration
    public static double KVA = 2;// Volts / rad/s or Volts per velocity
    public static double KSA = 0.0167040487354565;// Volts (required to start moving)

    //ROBOT DIMENSIONS
    public static double ROBOT_WIDTH = 300;//mm specifically between wheels, not full dt
    public static double ROBOT_LENGTH = 275;//mm specifically between wheels, not full dt
    public static double ROTATION_FACTOR = (ROBOT_WIDTH / 2) + (ROBOT_LENGTH / 2);
    public static double ROBOT_RADIUS = 420.0; // mm - distance from center to wheels
    public static double WHEEL_RADIUS = 52.0; // mm - adjust to your wheels
    public static double MOTOR_MAX_RPM = 435; // rpm at output shaft

    //ROBOT WHEEL POLARITIES
    public static int FRONT_RIGHT_POLARITY = 1;
    public static int FRONT_LEFT_POLARITY = 1;
    public static int BACK_RIGHT_POLARITY = -1;
    public static int BACK_LEFT_POLARITY = -1;

    //LQR MATRICES
    //estimated loop time avg 80hz
    public static double dt = 0.0125;
    public static double[][] A = {
            {1, 0, 0, dt, 0, 0},  // ex_dot = vx_robot - vx_desired
            {0, 1, 0, 0, dt, 0},  // ey_dot = vy_robot - vy_desired
            {0, 0, 1, 0, 0, dt},  // eθ_dot = ω_robot - ω_desired
            {0, 0, 0, 1, 0, 0},  // evx_dot = 0 (no acceleration model)
            {0, 0, 0, 0, 1, 0},  // evy_dot = 0
            {0, 0, 0, 0, 0, 1}   // evθ_dot = 0
    };

    public static double[][] B = {
            {0, 0, 0},  // vx correction affects ex
            {0, 0, 0},  // vy correction affects ey
            {0, 0, 0},  // ω correction affects eθ
            {dt, 0, 0},  // no direct effect on evx (but could model dynamics)
            {0, dt, 0},  // no direct effect on evy
            {0, 0, dt}   // no direct effect on evθ
    };

    // Q matrix (6x6) - cost on state errors
    public static double[][] Q = {
            {0.01, 0, 0, 0, 0, 0},  // ex cost
            {0, 0.1, 0, 0, 0, 0}, // ey cost (higher - prioritize lateral correction)
            {0, 0, .2, 0, 0, 0},  // eθ cost (high because error is in radians)
            {0, 0, 0, 0.0005, 0, 0},  // evx cost
            {0, 0, 0, 0, 0.0005, 0},  // evy cost
            {0, 0, 0, 0, 0, 0.001}  // evθ cost
    };

    // R matrix (3x3) - cost on control efforts
    public static double[][] R = {
            {0.00004, 0, 0},    // vx control cost
            {0, 0.00004, 0},    // vy control cost (slightly lower to allow more aggressive sideways movement
            {0, 0, 0.00001}    // ω control cost (mesured in r/s so looks bigger)
    };
}