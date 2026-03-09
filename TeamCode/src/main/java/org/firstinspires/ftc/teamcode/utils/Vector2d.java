package org.firstinspires.ftc.teamcode.utils;

public class Vector2d {
    private double x;
    private double y;
    private double magnitude;
    private double direction; // Radians

    // Constructors
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
        updateMagnitudeAndDirection();
    }

    public static Vector2d fromPolar(double magnitude, double directionRadians) {
        double x = magnitude * Math.cos(directionRadians);
        double y = magnitude * Math.sin(directionRadians);
        return new Vector2d(x, y);
    }

    // Getters
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return magnitude;
    }

    public double getDirection() {
        return direction;
    }

    public double getDirectionDegrees() {
        return Math.toDegrees(direction);
    }

    // Setters
    public void setX(double x) {
        this.x = x;
        updateMagnitudeAndDirection();
    }

    public void setY(double y) {
        this.y = y;
        updateMagnitudeAndDirection();
    }

    public void setMagnitude(double magnitude) {
        if (magnitude < 0) {
            throw new IllegalArgumentException("Magnitude cannot be negative.");
        }
        this.magnitude = magnitude;
        updateXY();
    }

    public void setDirection(double directionRadians) {
        this.direction = normalizeAngle(directionRadians);
        updateXY();
    }

    public void setDirectionDegrees(double directionDegrees) {
        setDirection(Math.toRadians(directionDegrees));
    }

    // Private helper methods to keep x/y and magnitude/direction in sync
    private void updateMagnitudeAndDirection() {
        this.magnitude = Math.sqrt(x * x + y * y);
        // Avoid NaN for zero vector, default direction to 0
        if (this.magnitude == 0) {
            this.direction = 0;
        } else {
            this.direction = Math.atan2(y, x);
        }
    }

    private void updateXY() {
        this.x = magnitude * Math.cos(direction);
        this.y = magnitude * Math.sin(direction);
    }

    // Vector operations

    /**
     * Adds another vector to this vector.
     *
     * @param other The vector to add.
     * @return A new Vector2d representing the sum.
     */
    public Vector2d add(Vector2d other) {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtracts another vector from this vector.
     *
     * @param other The vector to subtract.
     * @return A new Vector2d representing the difference.
     */
    public Vector2d subtract(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    /**
     * Scales this vector by a scalar value.
     *
     * @param scalar The scalar value to multiply by.
     * @return A new Vector2d representing the scaled vector.
     */
    public Vector2d scale(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    /**
     * Calculates the dot product of this vector and another vector.
     *
     * @param other The other vector.
     * @return The dot product.
     */
    public double dot(Vector2d other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Calculates the 2D cross product (magnitude of the 3D cross product).
     * This is useful for determining orientation or signed area.
     *
     * @param other The other vector.
     * @return The scalar value of the 2D cross product.
     */
    public double cross(Vector2d other) {
        return this.x * other.y - this.y * other.x;
    }

    /**
     * Rotates the vector by a given angle in radians.
     *
     * @param angleRadians The angle to rotate by, in radians.
     * @return A new Vector2d representing the rotated vector.
     */
    public Vector2d rotate(double angleRadians) {
        double newDirection = normalizeAngle(this.direction + angleRadians);
        return Vector2d.fromPolar(this.magnitude, newDirection);
    }

    /**
     * Rotates the vector by a given angle in degrees.
     *
     * @param angleDegrees The angle to rotate by, in degrees.
     * @return A new Vector2d representing the rotated vector.
     */
    public Vector2d rotateDegrees(double angleDegrees) {
        return rotate(Math.toRadians(angleDegrees));
    }

    /**
     * Returns a normalized version of this vector (unit vector).
     * If the magnitude is zero, it returns a zero vector.
     *
     * @return A new Vector2d with magnitude 1 and the same direction, or a zero vector.
     */
    public Vector2d normalize() {
        if (magnitude == 0) {
            return new Vector2d(0, 0); // Or throw an exception, depending on desired behavior
        }
        return new Vector2d(x / magnitude, y / magnitude);
    }

    /**
     * Calculates the angle between this vector and another vector.
     *
     * @param other The other vector.
     * @return The angle in radians (between 0 and PI).
     */
    public double angleTo(Vector2d other) {
        if (this.magnitude == 0 || other.magnitude == 0) {
            return 0; // Or handle as an error/NaN
        }
        double dotProduct = this.dot(other);
        double cosTheta = dotProduct / (this.magnitude * other.magnitude);
        // Clamp cosTheta to avoid floating point inaccuracies leading to Math.acos domain errors
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));
        return Math.acos(cosTheta);
    }

    /**
     * Calculates the distance between the point represented by this vector and another point.
     *
     * @param other The other vector representing a point.
     * @return The distance.
     */
    public double distanceTo(Vector2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Projects this vector onto another vector.
     *
     * @param other The vector to project onto.
     * @return A new Vector2d representing the projection.
     */
    public Vector2d projectOnto(Vector2d other) {
        if (other.magnitude == 0) {
            return new Vector2d(0, 0); // Cannot project onto a zero vector
        }
        double dotProduct = this.dot(other);
        double otherMagSq = other.magnitude * other.magnitude;
        return other.scale(dotProduct / otherMagSq);
    }

    /**
     * Returns a vector perpendicular to this one (rotated 90 degrees counter-clockwise).
     *
     * @return A new perpendicular Vector2d.
     */
    public Vector2d perpendicular() {
        return new Vector2d(-y, x);
    }

    /**
     * Linearly interpolates between this vector and another vector.
     *
     * @param other The target vector.
     * @param t     The interpolation factor (0.0 means this vector, 1.0 means other vector).
     * @return A new Vector2d representing the interpolated vector.
     */
    public Vector2d lerp(Vector2d other, double t) {
        return new Vector2d(
                this.x + (other.x - this.x) * t,
                this.y + (other.y - this.y) * t
        );
    }

    /**
     * Normalizes an angle to be within the range of -PI to PI radians.
     *
     * @param angle The angle in radians.
     * @return The normalized angle in radians.
     */
    public static double normalizeAngle(double angle) {
        // Reduce the angle to be within the range [0, 2*PI)
        angle = angle % (2 * Math.PI);

        // Adjust the angle to be within (-PI, PI]
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        } else if (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}