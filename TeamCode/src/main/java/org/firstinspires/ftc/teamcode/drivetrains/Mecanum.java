package org.firstinspires.ftc.teamcode.drivetrains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DataStorage;
import org.firstinspires.ftc.teamcode.auto.follower.LQR.Follower;
import org.firstinspires.ftc.teamcode.auto.follower.LQR.HeadingMode;
import org.firstinspires.ftc.teamcode.auto.follower.LQR.Path;
import org.firstinspires.ftc.teamcode.auto.follower.LQR.SplineGenerator;
import org.firstinspires.ftc.teamcode.utils.MotorControl;

import java.util.ArrayList;
import java.util.Arrays;

import org.firstinspires.ftc.teamcode.auto.follower.PurePursuit.PathFollower;
import org.firstinspires.ftc.teamcode.robotModules.Limelight;

@Config
public class Mecanum extends Drivetrain {
    public LinearOpMode opMode;

    public String color;

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;

    public GoBildaPinpointDriver odo;
    PathFollower dynamicPath = new PathFollower(this);
    Follower LQRPath;
    boolean isFollowingLQR = false;
    double savedX, savedY, savedH = 0;
    boolean isFollowingPath = false;
    public static double ODO_X_OFFSET = 0;
    public static double ODO_Y_OFFSET = 100;

    private double trackingHeading = 0;

    public Limelight limelight;

    private static final double FAST_DRIVE_DIVIDER = 1;
    private static final double SLOW_DRIVE_DIVIDER = 2.75;
    private static final double NORMAL_DRIVE_DIVIDER = 1.5;

    public MotorControl motorControl;

    private static final double MAX_FORWARD_VELOCITY = 1800; //mm/s
    private static final double MAX_STRAFING_VELOCITY = 1000; //mm/s

    private static final double MAX_POWER = .5; //power
    private boolean DEBUG_MODE = false;
    private boolean fieldCentric = true;

    private double limeX, limeY, limeT;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Mecanum(LinearOpMode opMode) {
        super("Mecanum");
        this.opMode = opMode;
        initMotors();
        initOdo();
        initLimelight();
    }

    public Mecanum(LinearOpMode opMode, boolean useOdo, boolean useLimelight) {
        super("Mecanum");
        this.opMode = opMode;
        initMotors();
        if (useOdo) {
            initOdo();
        } else {
            fieldCentric = false;
            odo = null;
        }

        if (useLimelight) {
            initLimelight();
        } else {
            limelight = null;
        }
    }

    public void turnOnDebugMode() {
        DEBUG_MODE = true;
    }

    public void update() {
        updateOdo();
        updateLimelightPos();
        graphRobotPosition();
        if (!isFollowingLQR) {
            if (fieldCentric) {
                applyGamepadToDriveMotorsFieldCentric();
            } else {
                applyGamepadToDriveMotors();
            }
            //
            //lockOnTarget();
        } else {
            if (!LQRPath.isDone() && Math.abs(opMode.gamepad1.left_stick_y) < 0.2 && Math.abs(opMode.gamepad1.left_stick_x) < 0.2 && Math.abs(opMode.gamepad1.right_stick_y) < 0.2 && Math.abs(opMode.gamepad1.right_stick_x) < 0.2) {
                LQRPath.update();
            } else {
                LQRPath.stop();
                isFollowingLQR = false;
            }
        }

//        applyGamepadInputToSavePosition();
//        applyGamepadInputToDriveToPosition();

    }

    public void updateForAuto() {
        updateOdo();
        //updateLimelightPos();
    }

    public void setColor(String color) {
        this.color = color;
        if (color.equals("RED")) {
            trackingHeading = -Math.PI/2; //90 deg in rad
        } else {
            trackingHeading = Math.PI/2;
        }
    }

    private void initMotors() {
        motorBackLeft = opMode.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = opMode.hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFrontLeft = opMode.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = opMode.hardwareMap.get(DcMotor.class, "motorFrontRight");

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initOdo() {
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //NEED TO CALCULATE TO CENTER OF ROBOT
        odo.setOffsets(ODO_X_OFFSET, ODO_Y_OFFSET);
        //sets resolution (ticks to mm)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // need to switch based on graph
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public void loadPosition() {
        Pose2D pose = DataStorage.getPose();
        odo.setPosition(pose);
    }

    public void resetOdoPos() {
        if (odo == null) return;
        odo.resetPosAndIMU();
    }

    private void initLimelight() {
        limelight = new Limelight(opMode);
    }

    private void resetMotorEncoders() {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void applyGamepadToDriveMotors() {
        double leftPadVertPower = opMode.gamepad1.left_stick_y;
        double leftPadHorizPower = opMode.gamepad1.left_stick_x;
        double rightPadHorizPower = opMode.gamepad1.right_stick_x;

        double strafeCorrectionFactor = MAX_FORWARD_VELOCITY / MAX_STRAFING_VELOCITY;

        leftPadHorizPower *= strafeCorrectionFactor;

        // Calculate wheel ratios (standard Mecanum drive equations)
        double frontLeftWheelPower = (leftPadVertPower - leftPadHorizPower - rightPadHorizPower);
        double frontRightWheelPower = -(leftPadVertPower + leftPadHorizPower + rightPadHorizPower); // Corrected sign for rotation
        double backLeftWheelPower = (leftPadVertPower + leftPadHorizPower - rightPadHorizPower);
        double backRightWheelPower = -(leftPadVertPower - leftPadHorizPower + rightPadHorizPower); // Corrected sign for rotation


        // Find |max| for normalization
        double largestValueOfMotors = Math.max(Math.abs(frontLeftWheelPower),
                Math.max(Math.abs(frontRightWheelPower),
                        Math.max(Math.abs(backLeftWheelPower),
                                Math.abs(backRightWheelPower))));

        // Normalize wheel powers to keep them within the [-1, 1] range
        // Only divide if the largest value is greater than 1 to prevent division by zero or unnecessary scaling
        if (largestValueOfMotors > 1.0) {
            frontLeftWheelPower /= largestValueOfMotors;
            frontRightWheelPower /= largestValueOfMotors;
            backLeftWheelPower /= largestValueOfMotors;
            backRightWheelPower /= largestValueOfMotors;
        }

        // Driver buttons change speed
        double motorDriverDivider;
        if (opMode.gamepad1.right_trigger > 0.5) // Check for trigger press
        {
            motorDriverDivider = FAST_DRIVE_DIVIDER;
        }
        else if (opMode.gamepad1.left_trigger > 0.5) // Check for trigger press
        {
            motorDriverDivider = SLOW_DRIVE_DIVIDER;
        }
        else
        {
            motorDriverDivider = NORMAL_DRIVE_DIVIDER;
        }

        // Set power of motors
        motorFrontLeft.setPower(frontLeftWheelPower / motorDriverDivider);
        motorFrontRight.setPower(frontRightWheelPower / motorDriverDivider);
        motorBackLeft.setPower(backLeftWheelPower / motorDriverDivider);
        motorBackRight.setPower(backRightWheelPower / motorDriverDivider);
    }

    private void applyGamepadToDriveMotorsFieldCentric() {
        double leftPadVertPower = -opMode.gamepad1.left_stick_y; // invert Y for typical joystick orientation
        double leftPadHorizPower = opMode.gamepad1.left_stick_x;
        double rightPadHorizPower = opMode.gamepad1.right_stick_x;

        double strafeCorrectionFactor = MAX_FORWARD_VELOCITY / MAX_STRAFING_VELOCITY;
        leftPadHorizPower *= strafeCorrectionFactor;

        // Get the robot’s current heading (radians)
        double botHeading = (-odo.getHeading() + trackingHeading);

        // Rotate joystick input for field-centric control
        double rotatedX = leftPadHorizPower * Math.cos(botHeading) - leftPadVertPower * Math.sin(botHeading);
        double rotatedY = leftPadHorizPower * Math.sin(botHeading) + leftPadVertPower * Math.cos(botHeading);

        // Mecanum wheel equations with rotated inputs
        double frontLeftWheelPower = -(rotatedY + rotatedX + rightPadHorizPower);
        double frontRightWheelPower = (rotatedY - rotatedX - rightPadHorizPower);
        double backLeftWheelPower = -(rotatedY - rotatedX + rightPadHorizPower);
        double backRightWheelPower = (rotatedY + rotatedX - rightPadHorizPower);

        // Normalize powers
        double max = Math.max(1.0, Math.max(Math.abs(frontLeftWheelPower),
                Math.max(Math.abs(frontRightWheelPower),
                        Math.max(Math.abs(backLeftWheelPower),
                                Math.abs(backRightWheelPower)))));

        frontLeftWheelPower /= max;
        frontRightWheelPower /= max;
        backLeftWheelPower /= max;
        backRightWheelPower /= max;

        // Speed modifiers
        double motorDriverDivider;
        if (opMode.gamepad1.right_trigger > 0.5) {
            motorDriverDivider = FAST_DRIVE_DIVIDER;
        } else if (opMode.gamepad1.left_trigger > 0.5) {
            motorDriverDivider = SLOW_DRIVE_DIVIDER;
        } else {
            motorDriverDivider = NORMAL_DRIVE_DIVIDER;
        }

        // Apply motor power
        motorFrontLeft.setPower(frontLeftWheelPower / motorDriverDivider);
        motorFrontRight.setPower(frontRightWheelPower / motorDriverDivider);
        motorBackLeft.setPower(backLeftWheelPower / motorDriverDivider);
        motorBackRight.setPower(backRightWheelPower / motorDriverDivider);
    }

    public void setTurnPower(double power) {
        setWheelPowers(power, power, power, power);
    }

    public void addTurnPower(double power) {
        double flP = motorFrontLeft.getPower();
        double frP = motorFrontRight.getPower();
        double blP = motorBackLeft.getPower();
        double brP = motorBackRight.getPower();
        setWheelPowers(flP + power, frP + power, blP + power, brP + power);
    }

    public void setWheelPowers(double fl, double fr, double bl, double br) {
        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);
    }

    public void updateOdo() {
        if (odo == null) return;
        odo.update();
    }

    public void updateLimelightPos() {
        double[] limePos = limelight.getXYTheta();

        //if they are all 0, return cause it cant see it
        if (limePos[0] == 0 && limePos[1] == 0 && limePos[2] == 0) {
            return;
        }

        limeX = limePos[0] * 1000;
        limeY = limePos[1] * 1000;
        limeT = limePos[2] * 1000;
    }

    public void setOdoPosition(Pose2D pos) {
        if (odo == null) return;
        odo.setPosition(pos);
    }

    public double getHeading() {
        updateOdo();
        return odo.getHeading();
    }

    public double getVelocity() {
        return Math.sqrt(Math.pow(odo.getVelX(), 2) + Math.pow(odo.getVelY(), 2));
    }

    private void applyGamepadInputToSavePosition() {
        if (opMode.gamepad1.a && opMode.gamepad1.x) {
            savedX = odo.getPosX();
            savedY = -odo.getPosY();
            savedH = Math.toDegrees(odo.getHeading());
        }
    }

    private void applyGamepadInputToDriveToPosition() {
        if (opMode.gamepad1.x && opMode.gamepad1.y) {
            updateOdo();
            ArrayList<Point> path = new ArrayList<Point>();
            path.add(new Point(odo.getPosX(), -odo.getPosY()));
            path.add(new Point(savedX, savedY));

            //dynamicPath.setPath(path, Math.toDegrees(odo.getHeading()), savedH);

            dynamicPath.startPath();
            isFollowingPath = true;
        }
    }

    public void graphRobotPosition() {
        TelemetryPacket packet = new TelemetryPacket();

        if (odo == null) {
            return;
        }

        packet.put("x", odo.getPosX());
        packet.put("y", odo.getPosY());
        packet.put("heading", Math.toDegrees(odo.getHeading()));
        packet.put("lime x", limeX);
        packet.put("lime y", limeY);
        packet.put("lime heading", Math.toDegrees(limeT));
        packet.put("isfollowing", isFollowingLQR);

        opMode.telemetry.addData("x: ", odo.getPosX());
        opMode.telemetry.addData("y: ", odo.getPosY());
        opMode.telemetry.addData("h: ", Math.toDegrees(odo.getHeading()));


        //convert x and y to inches
        packet.fieldOverlay()
            .setFill("red")
                .fillRect((odo.getPosX() / 25.4) - 7.5, (odo.getPosY() / 25.4) - 7.5, 15, 15);

        //covert to inches
        packet.fieldOverlay()
                .setFill("yellow")
                .fillRect((limeX / 25.4) - 7.5, (limeY / 25.4) - 7.5, 15, 15);

        dashboard.sendTelemetryPacket(packet);

    }

    public void lockOnTarget() {
        double P = 0.9;
        double[] XZA = limelight.getXZAFromTarget(); //X is side to side, Z is forward, A is yaw

        double x = XZA[0];
        double z = XZA[1];
        double a = XZA[2];

        //if they are all 0, return
        if (x == 0 && z == 0 && a == 0) {
            return;
        }

        //calculate the angle the robot should be facing to face the target
        double angleToTarget = Math.atan2(x, z);
        //calculate the difference between the current heading and the angle to the target
        double angleDifference = angleToTarget - a;

        //rotate the robot to face the target
        if (Math.abs(angleDifference) > Math.toRadians(2)) {
            addTurnPower(-angleDifference * P);
        }
    }

    public void startDriveToSquare() {
        Point target;
        if (color.equals("RED")) {
            target = new Point(960, -820);
        } else {
            target = new Point(960, 820);
        }
        SplineGenerator generator = new SplineGenerator();

        // 2. Create a simple straight path (60 inches / ~1500mm forward)
        Path testPath = generator.createPath(
                Arrays.asList(new Point(odo.getPosX(), odo.getPosY()), target),
                odo.getHeading(), 0, HeadingMode.INTERPOLATED
        );

        LQRPath = new Follower(testPath, this);
        isFollowingLQR = true;
        LQRPath.start();
    }

    public double getBatteryVoltage() {
        return opMode.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
}