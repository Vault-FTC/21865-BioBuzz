package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Location;
import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivebase extends Subsystem {

    private final DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    GoBildaPinpointDriver Pinpoint;
    double headingOffsetThingy;

    double modify_joystick_rotate = 0.0;

    /**
     * The motor names and directions, and the Pinpoint name, pod offsets, pod directions and pod type, all come
     * from pedro/Constants.java (the AutoTune results), so there is one place to change them.
     */
    public Drivebase(HardwareMap hardwareMap) {
        MecanumConfig wheels = Constants.drivetrainConfig;
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, wheels.frontLeftName.get());
        frontRightMotor = hardwareMap.get(DcMotorEx.class, wheels.frontRightName.get());
        backLeftMotor = hardwareMap.get(DcMotorEx.class, wheels.backLeftName.get());
        backRightMotor = hardwareMap.get(DcMotorEx.class, wheels.backRightName.get());

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setDirection(wheels.frontLeftDirection.get());
        frontRightMotor.setDirection(wheels.frontRightDirection.get());
        backLeftMotor.setDirection(wheels.backLeftDirection.get());
        backRightMotor.setDirection(wheels.backRightDirection.get());

        // Pinpoint, set up exactly the way Pedro's PinpointLocalizer sets it up from the same config.
        PinpointConfig pods = Constants.localizerConfig;
        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pods.name.get());
        Pinpoint.setOffsets(pods.xPodOffset.get(), pods.yPodOffset.get(), pods.offsetUnits.get());
        if (pods.ticksPerUnit.get().isPresent()) {
            Pinpoint.setEncoderResolution(pods.ticksPerUnit.get().getAsDouble(), pods.encoderResolutionUnit.get());
        } else {
            Pinpoint.setEncoderResolution(pods.podType.get());
        }
        Pinpoint.setEncoderDirections(pods.xPodDirection.get(), pods.yPodDirection.get());
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

    /**
     * Field-relative stick drive. {@code headingOffset} is the field heading the stick should treat as
     * "forward" (see FieldConstants.RED_STICK_UP_HEADING_DEG / BLUE_STICK_UP_HEADING_DEG), so the same
     * stick direction always
     * moves the robot the same way on the field no matter which way it is facing.
     */
    public void drive(double forward, double right, double rotate, double headingOffset) {
        headingOffsetThingy = headingOffset;
        drive(forward, right, rotate + modify_joystick_rotate);
    }

    /**
     * Robot-relative drive. Positive {@code forward} moves the robot forward, positive {@code right}
     * strafes it to its right, and positive {@code rotate} turns clockwise. Uses the heading offset set
     * by the last call to {@link #drive(double, double, double, double)} (0 after a {@link #driveField} call).
     */
    public void drive(double forward, double right, double rotate) {
        double botHeading = -Pinpoint.getHeading(AngleUnit.RADIANS) + headingOffsetThingy;
        // Rotate the robot-relative command into the field frame using the current heading.
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
     * counter-clockwise (heading increasing).
     */
    public void driveField(double xPower, double yPower, double ccwTurnPower) {
        headingOffsetThingy = 0;   // field moves are absolute; drop any teleop stick offset
        // drive() is robot-relative: +forward is the robot's nose and +right is its right, so a field
        // +Y (left of a +X-facing robot) needs a negative "right" command.
        drive(xPower, -yPower, -ccwTurnPower);
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
        double xPower = distance > 1e-6 ? speed * xError / distance : 0;
        double yPower = distance > 1e-6 ? speed * yError / distance : 0;
        double turnPower = Math.max(Math.min(headingError * TURN_P, TURN_MAX_POWER), -TURN_MAX_POWER);

        if (telemetry != null) {
            telemetry.addData("Target", "X: %.1f  Y: %.1f  H: %.1f", target.x, target.y, target.headingDegrees);
            telemetry.addData("Error", "X: %.1f  Y: %.1f  H: %.1f", xError, yError, headingError);
        }

        driveField(xPower, yPower, turnPower);
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