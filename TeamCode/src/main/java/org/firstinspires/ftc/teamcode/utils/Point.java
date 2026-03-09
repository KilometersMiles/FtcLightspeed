package org.firstinspires.ftc.teamcode.utils;

public class Point {
    public double x, y, heading;

    public Point(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
        this.heading = 0;
    }

    public double distance(Point otherPoint) {
        return Math.sqrt(Math.pow(otherPoint.x - this.x, 2) + Math.pow(otherPoint.y - this.y, 2));
    }

    public String toString () {
        return "(" + this.x + ", " + this.y + ", " + this.heading + ")";
    }
}