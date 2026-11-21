package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Shared park-only route: drive straight from the start to the park spot. Each park auto (for example
 * BlueParkCornerAuto) extends this and only says where it starts, by overriding startPose().
 * Everything else (setup, loops, telemetry, the universal timeout) comes from BlueAutoTemplate.
 */
public abstract class BlueParkTemplate extends BlueAutoTemplate {

    protected final Pose park = poseFactory.of(127.5, 45.5, 180);

    @Override
    protected Command autoRoutine() {
        return sequential(
                new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED)),   // flywheels on for the whole auto
                move(park())
        );
    }

    public Path park() {
        return named("park", line(startPose(), park).linear(startPose(), park));
    }
}
