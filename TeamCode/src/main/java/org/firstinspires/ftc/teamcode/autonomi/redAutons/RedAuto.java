package org.firstinspires.ftc.teamcode.autonomi.redAutons;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandSystem.Command;

// Preload shot, then park.
// Everything shared (setup, loops, telemetry, move/shoot helpers, the universal timeout) is in RedAutoTemplate.
@Autonomous(name = "Red Auto", group = "Red")
public class RedAuto extends RedAutoTemplate {

    // Poses (protected so an auto that extends this one can use them)
    protected final Pose start = poseFactory.of(59, 8.5, 90);
    protected final Pose park = poseFactory.of(11, 95, 90);
    protected final Pose parkControl1 = poseFactory.of(20.2071, 37.4207, 0);

    @Override
    protected Pose startPose() {
        return start;
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
