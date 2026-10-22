package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

// IMPORTS
import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.commandSystem.DelayCommand;
import org.firstinspires.ftc.teamcode.commandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Blue Auto", group = "Blue")
public class BlueAuto extends LinearOpMode {

    // CLASS-WIDE VARIABLES: constants, pose definitions and subsystems go here
        // Shot speeds for this auto. They match the shooter's defaults now. Change them here to override the
    // shooter's speeds for this auto only.
    static final double POLLEN_SPEED = Shooter.POLLEN_SPEED;
    static final double NECTAR_SPEED = Shooter.NECTAR_SPEED;

    // How long a shot keeps feeding once the gates are open (ms)
    static final double SHOT_FEED_MS = 1500;

    // Universal step timeout (ms): any step in timedRoute(...) that hasn't finished by then is cancelled and the
    // route moves on. 0 = off (no universal timeout).
    static final double STEP_TIMEOUT_MS = 0;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorAroundPoint(70.75, 70.75);

    // Pose definitions
    private final Pose start = poseFactory.of(59, 8.5, 90);
    private final Pose park = poseFactory.of(11, 95, 90);
    private final Pose parkControl1 = poseFactory.of(20.2071, 37.4207, 0);

    // Subsystems (declare here, create them in runOpMode)
    private FollowerSubsystem follower;
    private Intake intake;
    private Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        // LOCAL VARIABLES (only used inside runOpMode) go here

        // 1. Reset the scheduler (always first)
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.clearRegistry();

        // 2. Create subsystems here (Intake before Shooter)
        follower = new FollowerSubsystem(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, gamepad1);

        // 3. Subsystem settings go here
        shooter.setRumbleEnabled(false);
        shooter.setTargetSpeeds(POLLEN_SPEED, NECTAR_SPEED);

        // 4. Set the start pose
        sleep(300);
        follower.setPose(start);

        // 5. Path definitions go here

        // 6. Route: sequential commands go here (path moves, shots, waits), in the order they run
        Command route = timedRoute(
                shootCommand(),
                new FollowPathCommand(follower, park())
        );

        // 7. Init loop: init telemetry goes here
        while (!isStarted() && !isStopRequested()) {
            scheduler.run();
            telemetry.addData("Pose", "X: %.1f  Y: %.1f  H: %.1f",
                    follower.getPose().x(), follower.getPose().y(), Math.toDegrees(follower.getPose().heading()));
            telemetry.update();
        }
        if (isStopRequested()) return;

        // 8. Start the route
        scheduler.schedule(route);

        // 9. Main loop: per-loop telemetry goes here
        while (opModeIsActive()) {
            scheduler.run();
            telemetry.addData("Route", route.scheduled() ? "running" : "done");
            telemetry.update();
        }

        // 10. Cleanup
        follower.stop();
    }

    // HELPER METHODS: path methods and command methods go here

    public Path park() {
        return curve(start, parkControl1, park).linear(start, park);
    }

    // Shot at this auto's speeds (POLLEN_SPEED / NECTAR_SPEED above), waits as long as it takes to reach speed
    private Command shootCommand() {
        return shootCommand(POLLEN_SPEED, NECTAR_SPEED, 0);
    }

    // Shot at this auto's speeds. If the flywheels aren't at speed after timeoutMs, it feeds anyway.
    // For example shootCommand(4000)
    private Command shootCommand(double timeoutMs) {
        return shootCommand(POLLEN_SPEED, NECTAR_SPEED, timeoutMs);
    }

    // Shot at custom speeds (pollen, nectar) for this one shot only, for example shootCommand(1100, 1300)
    private Command shootCommand(double pollenSpeed, double nectarSpeed) {
        return shootCommand(pollenSpeed, nectarSpeed, 0);
    }

    // Shot at custom speeds that feeds anyway after timeoutMs if the flywheels aren't at speed, for example
    // shootCommand(1100, 1300, 4000). A timeout of 0 or less means no timeout.
    private Command shootCommand(double pollenSpeed, double nectarSpeed, double timeoutMs) {
        return new ShootCommand(shooter, intake, pollenSpeed, nectarSpeed, timeoutMs, SHOT_FEED_MS);
    }

    // Builds the route as a sequence. If STEP_TIMEOUT_MS is above 0, every step also gets that timeout.
    private Command timedRoute(Command... steps) {
        if (STEP_TIMEOUT_MS <= 0) {
            return new SequentialCommandGroup(steps);
        }
        Command[] timed = new Command[steps.length];
        for (int i = 0; i < steps.length; i++) {
            timed[i] = steps[i].withTimeout(STEP_TIMEOUT_MS);
        }
        return new SequentialCommandGroup(timed);
    }
}
