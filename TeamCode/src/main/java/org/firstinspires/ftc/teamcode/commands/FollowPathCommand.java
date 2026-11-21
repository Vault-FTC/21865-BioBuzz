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
    private final String name;

    public FollowPathCommand(FollowerSubsystem followerSubsystem, Path path) {
        this(followerSubsystem, path, "path");
    }

    /** name is shown on telemetry while this path runs (see FollowerSubsystem.getCurrentPathName()). */
    public FollowPathCommand(FollowerSubsystem followerSubsystem, Path path, String name) {
        this.followerSubsystem = followerSubsystem;
        this.path = path;
        this.name = name;
        addRequirements(followerSubsystem);
    }

    @Override
    public void initialize() {
        followerSubsystem.setCurrentPathName(name);
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
        followerSubsystem.setCurrentPathName(name + (interrupted ? " (stopped)" : " (done)"));
    }
}
