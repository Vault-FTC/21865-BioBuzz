package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.autonomi.Location;
import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;

public class Drivebase extends Subsystem {

    private final DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    GoBildaPinpointDriver Pinpoint;
    double headingOffsetThingy;

    double modify_joystick_rotate = 0.0;

    public Drivebase(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// Reverse one side of motors if needed (depends on robot configuration)
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
// Odometry constants and such
        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        //odo.setOffsets(205.71207, -15.175, DistanceUnit.MM);
        Pinpoint.setOffsets(0.0, 31.5, DistanceUnit.MM); // x-pod 0 mm, y-pod 31.5 mm
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //  odo.resetPosAndIMU();
    }

    public void setToCoastMode() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void resetHeading(double headingDegrees) {
        Pinpoint.setHeading(headingDegrees, AngleUnit.DEGREES);
        //odo.resetPosAndIMU();
    }

    public Pose2D getPosition() {
        return Pinpoint.getPosition();
    }

    public GoBildaPinpointDriver getPinpoint()
    {
        return Pinpoint;
    }



    public String getPositionTelemetry() {
        return String.format("X: %.1f in  Y: %.1f in  Heading: %.1f deg",
                Pinpoint.getPosX(FieldConstants.DISTANCE_UNIT),
                Pinpoint.getPosY(FieldConstants.DISTANCE_UNIT),
                Pinpoint.getHeading(AngleUnit.DEGREES));
    }

    public void updateAutoAim(double joystick_rx_modifier)
    {
        modify_joystick_rotate = joystick_rx_modifier;
    }

    public void drive(double forward, double right, double rotate, double headingOffset) {
        headingOffsetThingy = headingOffset;
        drive(forward, right, rotate + modify_joystick_rotate);
    }

    // The robot's translation was coming out reversed (forward/back and left/right) with rotation correct,
    // so the translation inputs are flipped here. Set to 1 if that's ever fixed at the source.
    private static final double TRANSLATION_SIGN = -1;

    public void drive(double forward, double right, double rotate) {
        forward *= TRANSLATION_SIGN;
        right *= TRANSLATION_SIGN;
        double botHeading = -Pinpoint.getHeading(AngleUnit.RADIANS) + headingOffsetThingy;
//         X is positive up, Y is positive to the right
        double rotRight = right * Math.cos(botHeading) - forward * Math.sin(botHeading);
        double rotForward = right * Math.sin(botHeading) + forward * Math.cos(botHeading);

        double frontLeftPower = rotForward + rotRight + rotate;
        double backLeftPower = rotForward - rotRight + rotate;
        double frontRightPower = rotForward - rotRight - rotate;
        double backRightPower = rotForward + rotRight - rotate;

        // Calculate motor powers

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        double SpeedLimit = 1;

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }
        frontLeftPower *= SpeedLimit;
        backLeftPower *= SpeedLimit;
        frontRightPower *= SpeedLimit;
        backRightPower *= SpeedLimit;

        frontLeftPower = Math.abs(frontLeftPower) < 0.02 ? 0 : frontLeftPower;
        frontRightPower = Math.abs(frontRightPower) < 0.02 ? 0 : frontRightPower;
        backLeftPower = Math.abs(backLeftPower) < 0.02 ? 0 : backLeftPower;
        backRightPower = Math.abs(backRightPower) < 0.02 ? 0 : backRightPower;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    // Gains for driveToPosition. Distances are in field inches (see FieldConstants).
    private static final double DRIVE_P = 0.5;             // power per inch of error (was 0.2 per cm)
    private static final double DRIVE_MIN_POWER = 0.25;    // enough to overcome friction
    private static final double DRIVE_MAX_POWER = 0.4;
    private static final double DRIVE_DEADBAND_IN = 1.2;   // below this, stop forcing min power (was 3 cm)
    private static final double TURN_P = 0.015;            // power per degree of error
    private static final double TURN_MAX_POWER = 0.4;

    /**
     * Drives in the field frame. xPower moves toward +X, yPower toward +Y, and ccwTurnPower turns
     * counter-clockwise (heading increasing). This hides the sign conventions of drive():
     * a positive "forward" argument there moves toward -X, a positive "right" argument moves
     * toward +Y, and a positive rotate turns clockwise.
     */
    public void driveField(double xPower, double yPower, double ccwTurnPower) {
        headingOffsetThingy = 0;   // field moves are absolute; drop any teleop stick offset
        drive(-xPower, yPower, -ccwTurnPower);
    }

    /** Runs one control step toward a field target. Call every loop after update(). */
    public void driveToPosition(Location target, Telemetry telemetry) {
        double xError = target.x - Pinpoint.getPosX(FieldConstants.DISTANCE_UNIT);
        double yError = target.y - Pinpoint.getPosY(FieldConstants.DISTANCE_UNIT);
        double headingError = FieldConstants.normalizeDegrees(
                target.headingDegrees - Pinpoint.getHeading(AngleUnit.DEGREES));
        double distance = Math.hypot(xError, yError);

        // Scale the whole translation vector, not each axis, so the robot drives in a straight line.
        double speed = Math.min(distance * DRIVE_P, DRIVE_MAX_POWER);
        if (distance > DRIVE_DEADBAND_IN) {
            speed = Math.max(speed, DRIVE_MIN_POWER);
        }
//        double xPower = distance > 1e-6 ? speed * xError / distance : 0;
//        double yPower = distance > 1e-6 ? speed * yError / distance : 0;

        double xPower = Math.min(0.2 ,xError * DRIVE_P);
        double yPower = Math.min(0.2, yError * DRIVE_P);
        double turnPower = 0; // Math.max(Math.min(headingError * TURN_P, TURN_MAX_POWER), -TURN_MAX_POWER);

        if (telemetry != null) {
            telemetry.addData("Target", "X: %.1f  Y: %.1f  H: %.1f", target.x, target.y, target.headingDegrees);
            telemetry.addData("Error", "X: %.1f  Y: %.1f  H: %.1f", xError, yError, headingError);
        }

        driveField(-xPower, yPower, turnPower);
    }
    public void brake() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void end() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean isAtPosition(Location target) {
        return isAtPosition(target, 6, 17.5);
    }

    /** toleranceXY is in field inches, toleranceAngle in degrees. */
    public boolean isAtPosition(Location target, double toleranceXY, double toleranceAngle) {
        double xError = target.x - Pinpoint.getPosX(FieldConstants.DISTANCE_UNIT);
        double yError = target.y - Pinpoint.getPosY(FieldConstants.DISTANCE_UNIT);
        double headingError = FieldConstants.normalizeDegrees(
                target.headingDegrees - Pinpoint.getHeading(AngleUnit.DEGREES));
        return Math.abs(xError) < toleranceXY &&
                Math.abs(yError) < toleranceXY &&
                Math.abs(headingError) < toleranceAngle;
    }

    public void setCurrentPose(Pose2D pos)
    {
        Pinpoint.setPosition(pos);
    }

    /** x and y in field inches. */
    public void setCurrentPose(double x, double y, double radians)
    {
        setCurrentPose(new Pose2D(FieldConstants.DISTANCE_UNIT, x, y, AngleUnit.RADIANS, radians));
    }

    /** x and y in field inches. */
    public void setCurrentPose(double x, double y)
    {
        Pinpoint.setPosX(x, FieldConstants.DISTANCE_UNIT);
        Pinpoint.setPosY(y, FieldConstants.DISTANCE_UNIT);
    }

    public void update() {
        Pinpoint.update(); // updates the odometry internally
    }

    public static double distanceToGoal(Pose2D robot, Pose2D goal) {
        double dx = robot.getX(DistanceUnit.CM) - goal.getX(DistanceUnit.CM);
        double dy = robot.getY(DistanceUnit.CM) - goal.getY(DistanceUnit.CM);
        return Math.hypot(dx, dy);
    }
    public static double angleToGoal(Pose2D robot, Pose2D goal) {
        double dx = goal.getX(DistanceUnit.CM) - robot.getX(DistanceUnit.CM);
        double dy = goal.getY(DistanceUnit.CM) - robot.getY(DistanceUnit.CM);
        double goalHeading = Math.atan2(dy, dx); // radians
        double robotHeading = robot.getHeading(AngleUnit.RADIANS);
        double error = goalHeading - robotHeading;

        // Normalize to [-π, π]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        return error;
    }

}