package org.firstinspires.ftc.teamcode.drivetrains;

import org.firstinspires.ftc.teamcode.utils.Point;

public class Localizer {
    double x, y, theta;
    
    public Localizer() {
    }
    
    public Point getPos() {
        return new Point (x, y, theta);
    }
}
