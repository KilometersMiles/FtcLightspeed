package org.firstinspires.ftc.teamcode.drivetrains;

public class SwerveModule {
    private DcMotorEx motor;
    private RTPAxon axon;

    public SwerveModule() {
        //init hardware
    }

    public void setSpeedAndDirection(double speed, double angle) {
        double angleError = Math.abs(angle - axon.getCurrentAngleRadians());
        if (angleError > Math.PI/2) {
            angle -= Math.PI;
            speed *= -1;
        }        

        axon.setTarget();
        //make sure speed is normalized
        motor.setPower(speed);
    }
}
