package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandSystem.Command;

// Preload shot, then park.
// Everything shared (setup, loops, telemetry, move/shoot helpers, the universal timeout) is in BlueAutoTemplate.
@Autonomous(name = "Blue Auto", group = "Blue")
public class BlueAuto extends BlueAutoTemplate {

    // Poses (protected so an auto that extends this one can use them)
    protected final Pose start = poseFactory.of(82.5, 133, 270);
    protected final Pose park = poseFactory.of(130.5, 46.5, 270);
    protected final Pose parkControl1 = poseFactory.of(121.2929, 104.0793, 180);

    @Override
    protected Pose startPose() {
        return start;
    }

    // This auto shoots at 1100 instead of Shooter.POLLEN_SPEED
    @Override
    protected double pollenSpeed() {
        return 1100;
    }

    // The whole auto, in the order it runs.
    @Override
    protected Command autoRoutine() {
        return sequential(
                shootCommand(0, 1500),   // preload shot (this auto feeds for 1.5 s)
                move(park())
        );
    }

    public Path park() {
        return named("park", curve(startPose(), parkControl1, park).linear(startPose(), park));
    }
}
