package org.firstinspires.ftc.teamcode.autonomi;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

/**
 * Minimal Pedro 3 + command-system example. Copy this file for the red/blue autos.
 *
 * <p>Pattern:
 * <ol>
 *   <li>{@code clearRegistry()} so the scheduler does not keep the previous OpMode's subsystems</li>
 *   <li>build subsystems (their constructors register them with the scheduler)</li>
 *   <li>set the starting pose (shared field frame: inches, radians, CCW-positive)</li>
 *   <li>build paths</li>
 *   <li>schedule a command group after {@code waitForStart()}</li>
 *   <li>call {@code CommandScheduler.run()} every loop; it updates the follower and runs commands</li>
 * </ol>
 */
@Autonomous(name = "Pedro Example Auto", group = "Examples")
public class PedroExampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().clearRegistry();

        FollowerSubsystem follower = new FollowerSubsystem(hardwareMap);

        Pose start = new Pose(60, 8, Math.toRadians(90));
        Pose park = new Pose(15, 99, Math.toRadians(90));
        follower.setPose(start);

        // A path must have a heading interpolation; .linear(...) turns to the end heading as it drives.
        Path leave = Paths.line(start, park).linear(start, park);

        waitForStart();
        if (isStopRequested()) return;

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(new FollowPathCommand(follower, leave)));

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            Pose pose = follower.getPose();
            telemetry.addData("X (in)", pose.x());
            telemetry.addData("Y (in)", pose.y());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
            telemetry.update();
        }

        follower.stop();
    }
}
