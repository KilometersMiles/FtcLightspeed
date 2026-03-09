package org.firstinspires.ftc.teamcode.drivetrains;

public class Drivetrain {
    String type;
    Localizer localizer;
    public Drivetrain () {
        localizer = null;
    }

    public Drivetrain (String typeOfDrive) {
        type = typeOfDrive;
    }

    public String getType() {
        return type;
    }

    public void setType(String typeOfDrive) {
        type = typeOfDrive;
    }

    public void update() {
    }
}