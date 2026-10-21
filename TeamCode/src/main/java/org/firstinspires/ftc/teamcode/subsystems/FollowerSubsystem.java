package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Owns the Pedro Pathing follower for autonomous.
 *
 * <p>The command scheduler calls {@link #periodic()} once per loop, which is where the follower
 * (and therefore the Pinpoint) gets updated. Keeping the update here means no OpMode can forget to
 * call it.
 */
public class FollowerSubsystem extends Subsystem {
    private final Follower follower;

    public FollowerSubsystem(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public Follower getFollower() {
        return follower;
    }

    /** Places the robot at {@code pose} in the shared field frame (inches and radians). */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /** Current field pose (inches and radians). */
    public Pose getPose() {
        return follower.pose();
    }

    /** Starts following {@code path}. The path must have a heading interpolation set. */
    public void follow(Path path) {
        follower.follow(path);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    /** Cuts drive power. Call when autonomous ends. */
    public void stop() {
        follower.stop();
    }

    @Override
    public void periodic() {
        follower.update();

        // Keep the auto -> teleop handoff live while autonomous runs.
        Pose pose = follower.pose();
        PoseStorage.currentPose = new Pose2D(
                FieldConstants.DISTANCE_UNIT,
                pose.x(),
                pose.y(),
                AngleUnit.RADIANS,
                pose.heading());
    }
}
