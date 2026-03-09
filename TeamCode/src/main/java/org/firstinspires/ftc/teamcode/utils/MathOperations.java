package org.firstinspires.ftc.teamcode.utils;

/**
 * Common mathematical utilities for robotics navigation and control.
 */
public class MathOperations {

    /**
     * Normalizes an angle to the range (-pi, pi].
     * This is critical for preventing "over-rotation" in PID loops.
     *
     * @param radians The angle to wrap.
     * @return The wrapped angle in radians.
     */
    public static double angleWrap(double radians) {
        double wrapped = radians % (2 * Math.PI);
        if (wrapped > Math.PI) wrapped -= 2 * Math.PI;
        if (wrapped <= -Math.PI) wrapped += 2 * Math.PI;
        return wrapped;
    }
    /**
     * Linearly interpolates between two values.
     * Useful for smoothing transitions between motor powers or positions.
     */
    public static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }

    /**
     * Clamps a value between a minimum and maximum bound.
     * Frequently used to ensure motor power stays within [-1, 1].
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Returns the Euclidean distance between two 2D points.
     */
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    /**
     * Calculates the magnitude (length) of a 2D vector.
     */
    public static double magnitude(double x, double y) {
        return Math.hypot(x, y);
    }

    /**
     * Maps a value from one range to another.
     * Example: Mapping a sensor value (0-1024) to a servo position (0-1).
     */
    public static double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}