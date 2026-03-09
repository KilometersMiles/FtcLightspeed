package org.firstinspires.ftc.teamcode.utils;

/**
 * Utility class for performing common angle operations,
 * such as normalization, conversion, and difference calculation.
 */
public final class AngleUtils {

    // Private constructor to prevent instantiation
    private AngleUtils() {
    }

    /**
     * Normalizes an angle to the range of [-PI, PI] radians.
     * This is essential for heading-based tracking to ensure that
     * angles calculated by accumulating movement or subtracting headings
     * are always consistent and minimize the distance of rotation.
     *
     * @param angle The angle in radians to normalize.
     * @return The normalized angle in radians, in the range [-PI, PI].
     */
    public static double normalizeAngle(double angle) {
        // Start by reducing the angle to the range [0, 2 * PI]
        double normalized = angle % (2 * Math.PI);

        // If the result is negative, add 2 * PI to bring it into the positive range
        if (normalized <= -Math.PI) {
            normalized += (2 * Math.PI);
        } else if (normalized > Math.PI) {
            normalized -= (2 * Math.PI);
        }

        return normalized;
    }
}