package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * The procedures that show up in AutoTune. There is no OpMode to select: build and deploy, connect to the
 * Robot Controller, and open http://192.168.43.1:10158 in a browser.
 *
 * <p>Run them in this order. Each tuner ends with a Java tab; paste it into Constants.java.
 * <ol>
 *   <li>Mecanum: wheel directions (robot on blocks, wheels free).</li>
 *   <li>Pinpoint: pod directions and offsets (push the robot by hand).</li>
 *   <li>Foresight: the path-following values (needs clear floor space).</li>
 *   <li>Tests: check the result (hold, line, curve, localization, driving).</li>
 * </ol>
 * Each of these reads the configs in Constants.java, so tune them in this order.
 *
 * <p>FRONT OF THE ROBOT: the SHOOTER end is the front. "Forward" in every tuner means toward the shooter end.
 */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    /** Checks the whole follower. The Driving and Localization tests also work before Foresight is tuned. */
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
