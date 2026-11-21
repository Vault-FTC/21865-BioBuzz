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
    // Pinpoint pod offsets, in INCHES (offsetUnits below is INCH), measured from the robot's center of
    // rotation (Pedro wants the center). Pinpoint convention, with the shooter end as the front:
    //   X_POD_OFFSET: how far LEFT of center the X (forward-rolling) pod is. Right of center = negative.
    //   Y_POD_OFFSET: how far FORWARD of center the Y (sideways-rolling) pod is. Behind center = negative.
    // Hand-measured from the center 2026-11-17:
    //   x = -(12 1/8 - 12 29/32 / 2) = -5.671875   (pod is right of center)
    //   y = 9 31/32 - 7 3/4         = +2.21875    (pod is toward the shooter)
    // The old tuner values (x +5.6206, y -2.4129) have the same sizes but opposite signs: the tuner ran
    // while the pod directions were still FORWARD, before both pods were set to REVERSED.
    // ---------------------------------------------------------------------------------------------
    public static final double X_POD_OFFSET_IN = -5.671875;
    public static final double Y_POD_OFFSET_IN = 2.21875;

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
    // inches (offsetUnits); positions and everything else Pedro reports are in inches too.
    // ---------------------------------------------------------------------------------------------
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT_NAME);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(X_POD_OFFSET_IN);
        c.yPodOffset.set(Y_POD_OFFSET_IN);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);   // Odometry Test 2026-11-16: both pods read backwards
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);   // the pod offsets above are in inches
    });

    // ---------------------------------------------------------------------------------------------
    // Foresight (the path-following algorithm), in inches. These are the Foresight tuner's results. To
    // update them, re-run the tuner and paste its Java tab over this whole block.
    // ---------------------------------------------------------------------------------------------
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23984813613508096);
                Controller secondaryTranslationalForward = Controller.proportional(0.08861747744168122);
                Controller primaryTranslationalLateral = Controller.proportional(0.37861870265673003);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13988949375360155);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010595443452489145));
                c.brake.set(Controller.proportionalFeedforward(0.009006126934615773));

                c.headingFeedback.set(Controller.proportional(4.4254139028617905));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06015186395782536, 0.005590313856821306));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10712154785269126, 0.07911232743868482));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001479920152080113, 0.0017798206906616946));

                c.maxAchievableForwardVelocity.set(92.82812896213719);
                c.maxAchievableStrafeVelocity.set(75.9819614050871);
                c.naturalForwardDeceleration.set(24.622478439537456);
                c.naturalStrafeDeceleration.set(65.73345316763948);
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
