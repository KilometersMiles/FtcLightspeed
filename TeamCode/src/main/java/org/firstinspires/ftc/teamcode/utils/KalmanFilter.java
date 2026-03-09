package org.firstinspires.ftc.teamcode.utils;

public class KalmanFilter {
    private double Q; // Process noise covariance
    private double R; // Measurement noise covariance
    private double P; // Estimation error covariance
    private double X; // Estimated state (distance)
    private double K; // Kalman gain

    // Constructor to initialize the Kalman filter
    public KalmanFilter(double Q, double R, double initialP, double initialX) {
        this.Q = Q;
        this.R = R;
        this.P = initialP;
        this.X = initialX;
    }

    // Predict the next state
    public void predict() {
        // Predict the state (distance) - assuming no control input
        // X_k = X_{k-1}
        // P_k = P_{k-1} + Q
        P = P + Q;
    }

    // Update the state estimate with a new measurement
    public void update(double measurement) {
        // Calculate the Kalman gain
        // K = P / (P + R)
        K = P / (P + R);

        // Update the state estimate with the measurement
        // X = X + K * (measurement - X)
        X = X + K * (measurement - X);

        // Update the estimation error covariance
        // P = (1 - K) * P
        P = (1 - K) * P;
    }

    // Get the current estimated state (distance)
    public double getState() {
        return X;
    }

    public void setProcessNoise(double num) {
        Q = num;
    }
    public void setMeasurementNoise(double num) {
        R = num;
    }
    public void setKalmanGain(double num) {
        K = num;
    }
}