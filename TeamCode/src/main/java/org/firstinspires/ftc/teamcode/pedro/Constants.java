package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3 setup for this robot, filled in with the results of AutoTune (see Tuning.java).
 *
 * <p>Drivebase (teleop) reads the motor names and directions and the Pinpoint settings from here too, so this is
 * the only place to change them.
 *
 * <p>FRONT OF THE ROBOT: the shooter end is the front. "Forward" and "left" in every tuner are relative to it.
 *
 * <p>Updating a value: edit it here by hand, or re-run the tuner (http://192.168.43.1:10158) and paste its
 * Java tab over the matching config below.
 */
public class Constants {
    // ---------------------------------------------------------------------------------------------
    // Hardware names (must match the Driver Hub configuration exactly)
    // ---------------------------------------------------------------------------------------------
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";
    public static final String PINPOINT_NAME = "pinpoint";

    // ---------------------------------------------------------------------------------------------
    // Pinpoint pod offsets, in millimeters.
    // The Pinpoint tuner measured 2.4348428681140812 in and 7.1141574889656125 in. These are those values
    // times 25.4, which is an exact conversion (1 inch is defined as exactly 25.4 mm).
    // ---------------------------------------------------------------------------------------------
    public static final double X_POD_OFFSET_MM = 61.84500885009766;
    public static final double Y_POD_OFFSET_MM = 180.69960021972653;

    // ---------------------------------------------------------------------------------------------
    // Drivetrain. The directions are from the Mecanum tuner.
    // ---------------------------------------------------------------------------------------------
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(FRONT_LEFT_MOTOR);
        c.frontRightName.set(FRONT_RIGHT_MOTOR);
        c.backLeftName.set(BACK_LEFT_MOTOR);
        c.backRightName.set(BACK_RIGHT_MOTOR);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // ---------------------------------------------------------------------------------------------
    // Localizer (goBILDA Pinpoint). The pod directions are from the Pinpoint tuner. Offsets are in
    // millimeters (offsetUnits); positions and everything else Pedro reports are in inches.
    // ---------------------------------------------------------------------------------------------
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT_NAME);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(X_POD_OFFSET_MM);
        c.yPodOffset.set(Y_POD_OFFSET_MM);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.MM);
    });

    // ---------------------------------------------------------------------------------------------
    // Foresight (the path-following algorithm), in inches. These are the Foresight tuner's results. To
    // update them, re-run the tuner and paste its Java tab over this whole block.
    // ---------------------------------------------------------------------------------------------
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23);
                Controller secondaryTranslationalForward = Controller.proportional(0.086);
                Controller primaryTranslationalLateral = Controller.proportional(0.33);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011899554894773715));
                c.brake.set(Controller.proportionalFeedforward(0.010114621660557658));

                c.headingFeedback.set(Controller.proportional(3.0));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06444450597460338, 0.005164668918400906));

                c.linearBrakeCoefficients.set(Matrix.diag(2.458970481975108, 0.20343081813934002));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.041477158743428215, -3.5860533788652357E-4));

                c.maxAchievableForwardVelocity.set(71.16618904094076);
                c.maxAchievableStrafeVelocity.set(65.73392095803197);
                c.naturalForwardDeceleration.set(28.669274906603505);
                c.naturalStrafeDeceleration.set(68.09313607512907);
            }
    );

    /** Builds the Follower used by autonomous (through FollowerSubsystem). */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
