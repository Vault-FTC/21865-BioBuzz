package org.firstinspires.ftc.teamcode.autonomi.redAutons;

// IMPORTS

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.commandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.FlowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.PanelsSupport;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Garden Red Auto", group = "Red")
public class GardenRedAuto extends LinearOpMode {

    // CLASS-WIDE VARIABLES: constants, pose definitions and subsystems go here
    // Shot speeds for this auto. They match the shooter's defaults now. Change them here to override the
    // shooter's speeds for this auto only.
    static final double POLLEN_SPEED = Shooter.POLLEN_SPEED;
    static final double NECTAR_SPEED = Shooter.NECTAR_SPEED;

    // How long a shot keeps feeding once the gates are open (ms)
    static final double SHOT_FEED_MS = 2500;

    // Intake time (ms) for the flower moves, for example move(flower1(), INTAKE_RUN_MS)
    static final double INTAKE_RUN_MS = 4000;

    // How long move(...) waits for the flower arm to finish lowering / raising (ms), unless a move gives its own time
    static final double ARM_DOWN_MS = 800;
    static final double ARM_UP_MS = 800;

    // Universal step timeout (ms): any step in timedRoute(...) that hasn't finished by then is cancelled and the
    // route moves on. 0 = off (no universal timeout).
    static final double STEP_TIMEOUT_MS = 5000;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Red poses: Blue's Garden poses turned 180 degrees around the field center (x and y flipped, heading + 180)
    private final Pose start = poseFactory.of(59, 8.5, 90);
    private final Pose shoot1 = poseFactory.of(59, 19.5, 90);
    private final Pose garden = poseFactory.of(7.5, 5.5, 0);
    private final Pose gardenControl1 = poseFactory.of(31.5, 5.5, 180);
    private final Pose pt1Shoot2 = poseFactory.of(28.5, 100.5, 270);
    private final Pose pt1Shoot2Control1 = poseFactory.of(26.5, 27.4957, 180);
    private final Pose pt2Shoot2 = poseFactory.of(58, 127.5, 270);
    private final Pose pt2Shoot2Control1 = poseFactory.of(61.5, 111.5, 180);
    private final Pose park = poseFactory.of(14, 96, 0);
    private final Pose parkControl1 = poseFactory.of(59.5, 111.5, 180);

    // Subsystems (declare here, create them in runOpMode)
    private FollowerSubsystem follower;
    private Intake intake;
    private Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = PanelsSupport.joined(telemetry);   // also show telemetry on Panels
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
                move(shoot1(), 250),
                shootCommand(),   // preload shot
                move(garden(), 1500),
                follow(follower, pt1Shoot2()),
                follow(follower, pt2Shoot2()),
                shootCommand(),
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

    // What the flower arm does during a move()
    private enum ArmMove { NONE, UP, DOWN }

    // A path move that can also run the intake and/or move the flower arm while it drives, so they cost no extra
    // time. Everything after the path is optional. The move finishes when the path, the intake and the arm are done.
    //   move(park())                           plain move
    //   move(garden(), 1250)                   intake runs for 1250 ms
    //   move(park(), ArmMove.UP)               arm goes up (waits ARM_UP_MS)
    //   move(park(), ArmMove.UP, 400)          arm goes up (waits 400 ms)
    //   move(flower1(), 4000, ArmMove.DOWN)    intake and arm (add a 4th number for the arm's wait time)
    private Command move(Path path) {
        return move(path, 0, ArmMove.NONE, 0);
    }

    private Command move(Path path, double intakeMs) {
        return move(path, intakeMs, ArmMove.NONE, 0);
    }

    private Command move(Path path, ArmMove arm) {
        return move(path, 0, arm, 0);
    }

    private Command move(Path path, ArmMove arm, double armMs) {
        return move(path, 0, arm, armMs);
    }

    private Command move(Path path, double intakeMs, ArmMove arm) {
        return move(path, intakeMs, arm, 0);
    }

    private Command move(Path path, double intakeMs, ArmMove arm, double armMs) {
        java.util.ArrayList<Command> parts = new java.util.ArrayList<>();
        parts.add(follow(follower, path));
        if (intakeMs > 0) parts.add(new RunIntakeCommand(intake, intakeMs));
        if (arm == ArmMove.UP) parts.add(new FlowerArmCommand(intake, true, armMs > 0 ? armMs : ARM_UP_MS));
        if (arm == ArmMove.DOWN) parts.add(new FlowerArmCommand(intake, false, armMs > 0 ? armMs : ARM_DOWN_MS));
        return new ParallelCommandGroup(parts.toArray(new Command[0]));
    }

    // Each path starts from the END pose of the path before it in autoRoutine(), so the heading and position carry
    // over. If you reorder or remove steps (for example drop pt2Shoot2 and go straight from pt1Shoot2 to park),
    // change that path's start pose to the end pose of whatever path now comes before it.
    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path garden() {
        return Paths.curve(shoot1, gardenControl1, garden).linear(shoot1, garden);
    }

    public Path pt1Shoot2() {
        return Paths.curve(garden, pt1Shoot2Control1, pt1Shoot2).linear(garden, pt1Shoot2);
    }

    public Path pt2Shoot2() {
        return Paths.curve(pt1Shoot2, pt2Shoot2Control1, pt2Shoot2).linear(pt1Shoot2, pt2Shoot2);
    }

    public Path park() {
        return Paths.curve(pt2Shoot2, parkControl1, park).linear(pt2Shoot2, park);
    }

    // Shot at this auto's speeds. Both numbers are optional, and 0 means "use the default":
    //   timeoutMs: if the flywheels aren't at speed after this long, it feeds anyway. 0 = wait as long as it takes.
    //   feedMs: how long it feeds once the gates are open. 0 = SHOT_FEED_MS.
    //   shootCommand()          no timeout, default feed time
    //   shootCommand(3000, 0)   timeout only
    //   shootCommand(0, 2000)   feed time only
    //   shootCommand(3000, 2000)  both
    private Command shootCommand() {
        return shootCommand(0, 0);
    }

    private Command shootCommand(double timeoutMs, double feedMs) {
        return shootCommand(POLLEN_SPEED, NECTAR_SPEED, timeoutMs, feedMs);
    }

    // Same at custom speeds (pollen, nectar) for this one shot only, for example shootCommand(1100, 1300, 0, 0)
    private Command shootCommand(double pollenSpeed, double nectarSpeed, double timeoutMs, double feedMs) {
        return new ShootCommand(shooter, intake, pollenSpeed, nectarSpeed, timeoutMs, feedMs > 0 ? feedMs : SHOT_FEED_MS);
    }

    // Builds the route as a sequence. If STEP_TIMEOUT_MS is above 0, every step also gets that timeout.
    private Command timedRoute(Command... steps) {
        if (STEP_TIMEOUT_MS <= 0) {
            return new SequentialCommandGroup(steps);
        }
        Command[] timed = new Command[steps.length];
        for (int i = 0; i < steps.length; i++) {
            double extraMs = 0;   // a shot with a longer feed gets that extra time added to its cap
            if (steps[i] instanceof ShootCommand) {
                extraMs = Math.max(0, ((ShootCommand) steps[i]).getFeedMs() - SHOT_FEED_MS);
            }
            timed[i] = steps[i].withTimeout(STEP_TIMEOUT_MS + extraMs);
        }
        return new SequentialCommandGroup(timed);
    }
}
