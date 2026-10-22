package org.firstinspires.ftc.teamcode.autonomi;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

/**
 * Pedro test for the red side: drive forward, strafe left, then drive back to the start, holding the start
 * heading the whole time. If everything is right, the robot ends where it began, so the "Error from start"
 * telemetry at the end tells you how accurate the localizer and follower are.
 *
 * <p>Place the robot at the start pose below, facing the way START_HEADING_DEG says (shooter end is the front).
 * Forward and left are worked out from that heading, so they stay correct if you change it.
 *
 * <p>All positions are in field inches, the same frame as FieldConstants and the Pedro visualizer.
 */
@Autonomous(name = "Pedro Test (Red)", group = "Test")
public class PedroTestAutoRed extends LinearOpMode {
    // Start pose from the Pedro visualizer (red side).
    static final double START_X_IN = 59;
    static final double START_Y_IN = 8.5;
    static final double START_HEADING_DEG = 90;   // 90 = facing up the field (+Y)

    // How far to drive. Start small the first time.
    static final double FORWARD_IN = 36;
    static final double LEFT_IN = 36;

    // Set to false to drive only forward and straight back, with no strafe or diagonal. Handy for telling a
    // sideways-motion problem apart from a general one.
    static final boolean INCLUDE_STRAFE = true;

    // Telemetry is only sent this often, so it doesn't slow the control loop. The "Loop rate" line shows how
    // fast the loop really runs (the Tests procedures run a bare loop with no telemetry at all).
    static final double TELEMETRY_INTERVAL_MS = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.clearRegistry();

        FollowerSubsystem follower = new FollowerSubsystem(hardwareMap);
        sleep(300);   // let the Pinpoint finish its IMU reset before we write a pose to it

        // Work out the three corners of the route from the start heading.
        double heading = Math.toRadians(START_HEADING_DEG);
        double forwardX = Math.cos(heading), forwardY = Math.sin(heading);
        double leftX = -Math.sin(heading), leftY = Math.cos(heading);

        Pose start = new Pose(START_X_IN, START_Y_IN, heading);
        Pose afterForward = new Pose(start.x() + FORWARD_IN * forwardX, start.y() + FORWARD_IN * forwardY, heading);
        Pose afterLeft = new Pose(afterForward.x() + LEFT_IN * leftX, afterForward.y() + LEFT_IN * leftY, heading);
        follower.setPose(start);

        // .constant(heading) keeps the same heading for the whole path, so the strafe really is a strafe.
        Command route;
        if (INCLUDE_STRAFE) {
            Path forward = Paths.line(start, afterForward).constant(heading);
            Path strafeLeft = Paths.line(afterForward, afterLeft).constant(heading);
            Path backToStart = Paths.line(afterLeft, start).constant(heading);
            route = new SequentialCommandGroup(
                    new FollowPathCommand(follower, forward),
                    new FollowPathCommand(follower, strafeLeft),
                    new FollowPathCommand(follower, backToStart));
        } else {
            Path forward = Paths.line(start, afterForward).constant(heading);
            Path backStraight = Paths.line(afterForward, start).constant(heading);
            route = new SequentialCommandGroup(
                    new FollowPathCommand(follower, forward),
                    new FollowPathCommand(follower, backStraight));
        }

        // Keep the pose live during init so you can check it before pressing start.
        ElapsedTime telemetryTimer = new ElapsedTime();
        ElapsedTime rateTimer = new ElapsedTime();
        int loops = 0;
        while (!isStarted() && !isStopRequested()) {
            scheduler.run();
            loops++;
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                Pose pose = follower.getPose();
                telemetry.addLine("Place the robot at the start pose (shooter end forward), then press start.");
                telemetry.addData("Expected start", "X: %.1f  Y: %.1f  H: %.0f", START_X_IN, START_Y_IN, START_HEADING_DEG);
                telemetry.addData("Pose", "X: %.1f  Y: %.1f  H: %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Loop rate", "%.0f loops/s", loops / rateTimer.seconds());
                telemetry.update();
                telemetryTimer.reset();
                rateTimer.reset();
                loops = 0;
            }
        }
        if (isStopRequested()) return;

        scheduler.schedule(route);

        // The route runs inside scheduler.run(), which also updates the follower. Nothing here ends the opmode;
        // the driver station's timer (or Stop) does.
        telemetryTimer.reset();
        rateTimer.reset();
        loops = 0;
        while (opModeIsActive()) {
            scheduler.run();
            loops++;

            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                Pose pose = follower.getPose();
                telemetry.addData("Status", route.scheduled() ? "Driving" : "Done");
                telemetry.addData("Pose", "X: %.1f  Y: %.1f  H: %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Loop rate", "%.0f loops/s", loops / rateTimer.seconds());
                if (!route.scheduled()) {
                    telemetry.addData("Error from start", "X: %+.2f  Y: %+.2f  H: %+.1f",
                            pose.x() - start.x(), pose.y() - start.y(),
                            Math.toDegrees(pose.heading() - start.heading()));
                }
                telemetry.update();
                telemetryTimer.reset();
                rateTimer.reset();
                loops = 0;
            }
        }

        follower.stop();
    }
}
