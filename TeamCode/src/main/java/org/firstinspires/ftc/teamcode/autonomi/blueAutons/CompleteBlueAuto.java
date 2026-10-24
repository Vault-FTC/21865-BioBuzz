package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

// IMPORTS
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

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
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Complete Blue Auto", group = "Blue")
public class CompleteBlueAuto extends LinearOpMode {

    // CLASS-WIDE VARIABLES: constants, pose definitions and subsystems go here
    // Shot speeds for this auto. They match the shooter's defaults now. Change them here to override the
    // shooter's speeds for this auto only.
    static final double POLLEN_SPEED = Shooter.POLLEN_SPEED;
    static final double NECTAR_SPEED = Shooter.NECTAR_SPEED;

    // How long a shot keeps feeding once the gates are open (ms)
    static final double SHOT_FEED_MS = 1500;

    // How long the intake runs while driving the garden and flower paths (ms)
    static final double INTAKE_RUN_MS = 1000;

    // Universal step timeout (ms): any step in timedRoute(...) that hasn't finished by then is cancelled and the
    // route moves on. 0 = off (no universal timeout).
    static final double STEP_TIMEOUT_MS = 0;

    // Blue starts in the opposite corner, so red's poses are rotated 180 degrees around the center of the field
    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorAroundPoint(70.75, 70.75);

    private final Pose start = poseFactory.of(59, 8.5, 90);
    private final Pose garden = poseFactory.of(10, 7.5, 0);
    private final Pose shoot2 = poseFactory.of(59, 133.5, 270);
    private final Pose shoot2Control1 = poseFactory.of(22.0, 125.0, 0);
    private final Pose shoot2Control2 = poseFactory.of(66.2193, 119.343, 0);
    private final Pose flower1 = poseFactory.of(47, 127, 90);
    private final Pose flower1Control1 = poseFactory.of(56.2387, 121.2095, 0);
    private final Pose shoot3 = poseFactory.of(59, 133.5, 270);
    private final Pose shoot3Control1 = poseFactory.of(60.0744, 118.8244, 0);
    private final Pose flower2 = poseFactory.of(12.771, 47.8317, 180);
    private final Pose flower2Control1 = poseFactory.of(71.9061, 132.9709, 0);
    private final Pose flower2Control2 = poseFactory.of(1.0251, 93.1901, 0);
    private final Pose flower2Control3 = poseFactory.of(39.1327, 43.127, 0);
    private final Pose shoot4 = poseFactory.of(59, 8.5, 90);
    private final Pose park = poseFactory.of(14, 96, 90);
    private final Pose parkControl1 = poseFactory.of(24.7201, 39.3366, 0);

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
        Command route = autoRoutine();

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
            telemetry.addData("Pose", "X: %.1f  Y: %.1f  H: %.1f",
                    follower.getPose().x(), follower.getPose().y(), Math.toDegrees(follower.getPose().heading()));
            telemetry.update();
        }

        // 10. Cleanup
        follower.stop();
    }

    // HELPER METHODS: path methods and command methods go here

    // The whole auto, in the order it runs. Flywheels spin up first, then a preload shot, then a shot after each shoot path.
    public Command autoRoutine() {
        return sequential(
                new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED)),   // flywheels on for the whole auto
                shootCommand(),   // preload shot
                followAndIntake(garden()),
                follow(follower, shoot2()),
                shootCommand(),
                armDown(),
                followAndIntake(flower1()),
                follow(follower, shoot3()),
                shootCommand(),
                armUp(),
                armDown(),
                followAndIntake(flower2()),
                follow(follower, shoot4()),
                shootCommand(),
                armUp(),
                follow(follower, park())
        );
    }

    // These two let the Pedro visualizer's exported autoRoutine() work as written.
    private Command sequential(Command... steps) {
        return timedRoute(steps);
    }

    private Command follow(FollowerSubsystem follower, Path path) {
        return new FollowPathCommand(follower, path);
    }

    // Drives the path with the intake running. Finishes when both the path and the intake are done.
    private Command followAndIntake(Path path) {
        return new ParallelCommandGroup(follow(follower, path), new RunIntakeCommand(intake, INTAKE_RUN_MS));
    }

    public Path garden() {
        return line(start, garden).linear(start, garden, 0.6);   // heading finishes turning at 60% of the move
    }

    private Command armDown() {
        return new InstantCommand(() -> intake.setFlowerArmUp(false));
    }

    private Command armUp() {
        return new InstantCommand(() -> intake.setFlowerArmUp(true));
    }

    public Path shoot2() {
        return curve(garden, shoot2Control1, shoot2Control2, shoot2).linear(garden, shoot2);
    }

    public Path flower1() {
        return curve(shoot2, flower1Control1, flower1).linear(shoot2, flower1);
    }

    public Path shoot3() {
        return curve(flower1, shoot3Control1, shoot3).linear(flower1, shoot3);
    }

    public Path flower2() {
        return curve(shoot3, flower2Control1, flower2Control2, flower2Control3, flower2).linear(shoot3, flower2);
    }

    public Path shoot4() {
        return line(flower2, shoot4).linear(flower2, shoot4);
    }

    public Path park() {
        return curve(shoot4, parkControl1, park).linear(shoot4, park);
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
