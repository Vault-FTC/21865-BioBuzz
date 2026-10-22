package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

/**
 * Follows one Pedro path. Finishes when the follower stops being busy (path done and settled).
 * The follower itself is updated by {@code FollowerSubsystem.periodic()}.
 */
public class FollowPathCommand extends Command {
    private final FollowerSubsystem followerSubsystem;
    private final Path path;

    public FollowPathCommand(FollowerSubsystem followerSubsystem, Path path) {
        this.followerSubsystem = followerSubsystem;
        this.path = path;
        addRequirements(followerSubsystem);
    }

    @Override
    public void initialize() {
        followerSubsystem.follow(path);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !followerSubsystem.isBusy();
    }

    /** If the command is cancelled (for example by a timeout), stop the follower so it doesn't keep driving the path. */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) followerSubsystem.stop();
    }
}
