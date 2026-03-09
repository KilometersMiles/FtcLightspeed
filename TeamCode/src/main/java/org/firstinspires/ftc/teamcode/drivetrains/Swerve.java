package org.firstinspires.ftc.teamcode.drivetrains;

public class Swerve extends Drivetrain {
    private double robotWidth = Constants.robotWidth;
    private double robotLength = Constants.robotLength;
    //4 swerve modules
    private SwerveModule wheel1;
    private SwerveModule wheel2;
    private SwerveModule wheel3;
    private SwerveModule wheel4;

    public Swerve (LinearOpMode opMode) {
        super("Swerve");
        //init hardware
        wheel1 = new SwerveModule(/*probably hardware thing*/);
        wheel2 = new SwerveModule(/*probably hardware thing*/);
        wheel3 = new SwerveModule(/*probably hardware thing*/);
        wheel4 = new SwerveModule(/*probably hardware thing*/);
    }

    public void setDrivetrainState(double xVel, double yVel, double hVel) {
        double A = xVel - (hVel * (robotLength / 2));
        double B = xVel + (hVel * (robotLength / 2));
        double C = yVel - (hVel * (robotWidth / 2));
        double D = yVel + (hVel * (robotWidth / 2));

        //wheel 1
        double speed1 = Math.sqrt((B * B) + (C * C));
        double angle1 = Math.atan2(B,C); //in radians
        //wheel 2
        double speed2 = Math.sqrt((B * B) + (D * D));
        double angle2 = Math.atan2(B,D); //in radians
        //wheel 3
        double speed3 = Math.sqrt((A * A) + (D * D));
        double angle3 = Math.atan2(A,D); //in radians
        //wheel 4
        double speed4 = Math.sqrt((A * A) + (C * C));
        double angle4 = Math.atan2(A,C); //in radians

        //apply to each swerve module
        wheel1.setSpeedAndDirection(speed1, angle1);
        wheel2.setSpeedAndDirection(speed2, angle2);
        wheel3.setSpeedAndDirection(speed3, angle3);
        wheel4.setSpeedAndDirection(speed4, angle4);
    }
}
