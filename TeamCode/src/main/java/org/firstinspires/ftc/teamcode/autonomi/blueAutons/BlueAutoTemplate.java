package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.FlowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PanelsSupport;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Shared base for every BLUE auto. It has everything the autos have in common: setting up the robot, the
 * init and main loops, telemetry, and the helpers (move, shootCommand, armDown, the universal timeout...).
 *
 * <p>An auto extends this and fills in only its own part:
 * <pre>
 *   &#64;Autonomous(name = "My Blue Auto", group = "Blue")
 *   public class MyBlueAuto extends BlueAutoTemplate {
 *       protected final Pose start = poseFactory.of(82.5, 133, 270);   // its poses
 *       ...
 *       protected Pose startPose() { return start; }                    // where it starts
 *       protected Command autoRoutine() { return sequential(...); }     // what it does
 *       public Path shoot1() { ... }                                    // its paths
 *   }
 * </pre>
 * This class is abstract, so it can't run by itself and doesn't show up on the Driver Station.
 */
public abstract class BlueAutoTemplate extends LinearOpMode {

    // ---------------------------------------------------------------------------------------------
    // UNIVERSAL SETTINGS: shared by every blue auto. Change them here to change them for all of them.
    // ---------------------------------------------------------------------------------------------

    // How long a shot keeps feeding once the gate is open (ms)
    static final double SHOT_FEED_MS = 1300;

    // How long armDown() / armUp() wait for the flower arm to finish moving before the next step starts (ms)
    static final double ARM_DOWN_MS = 650;
    static final double ARM_UP_MS = 800;

    // Universal step timeout (ms): any step in autoRoutine() that hasn't finished by then is cancelled (path
    // stopped, intake off) and the route moves on. 0 = off. A step gets a longer cap only when it's set up
    // to need one:
    //   - a move with a long intake/arm time: that time + STEP_MARGIN_MS
    //   - a shot with custom numbers whose spin-up wait + feed time is longer
    //   - any step wrapped in withStepTimeout(step, ms)
    static final double STEP_TIMEOUT_MS = 3000;
    static final double STEP_MARGIN_MS = 500;

    // With the universal timeout on, a shot that waits this long for the flywheel feeds anyway (ms), instead of
    // being cancelled by the timeout before it ever shoots. Used when a shot's own timeoutMs is 0.
    static final double SHOT_SPINUP_TIMEOUT_MS = 1500;

    // Makes poses from degrees: poseFactory.of(x inches, y inches, heading degrees)
    protected final PoseFactory poseFactory = PoseFactory.degrees();

    // Subsystems: created in runOpMode(), usable by every auto
    protected FollowerSubsystem follower;
    protected Intake intake;
    protected Shooter shooter;

    // ---------------------------------------------------------------------------------------------
    // WHAT EACH AUTO MUST FILL IN
    // ---------------------------------------------------------------------------------------------

    /** Where this auto starts. Override it in a copy of an auto to start the same route somewhere else. */
    protected abstract Pose startPose();

    /** The auto's steps, in the order they run. Usually return sequential(...). */
    protected abstract Command autoRoutine();

    /**
     * Fixed flywheel speed for this auto. It's what the flywheels hold while driving, and what shots use when the
     * LUT is off (Shooter.USE_LUT = false). Override to give one auto a different fixed speed.
     */
    protected double pollenSpeed() {
        return Shooter.POLLEN_SPEED;
    }

    /** true: shots without a custom speed pick their speed from the LUT. Override and return false to turn it off for one auto. */
    protected boolean useLut() {
        return Shooter.USE_LUT;
    }

    // ---------------------------------------------------------------------------------------------
    // RUNNING THE AUTO (the same for every auto)
    // ---------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = PanelsSupport.joined(telemetry);   // also show telemetry on Panels

        // 1. Reset the scheduler (always first)
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.clearRegistry();

        // 2. Create subsystems (Intake before Shooter)
        follower = new FollowerSubsystem(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, gamepad1);

        // 3. Subsystem settings
        shooter.setRumbleEnabled(false);
        shooter.setTargetSpeed(pollenSpeed());

        // 4. Set the start pose
        sleep(300);
        follower.setPose(startPose());

        // 5. Build the route (the auto's own autoRoutine())
        Command route = autoRoutine();

        // 6. Init loop
        while (!isStarted() && !isStopRequested()) {
            scheduler.run();
            telemetry.addData("Pose", "X: %.1f  Y: %.1f  H: %.1f",
                    follower.getPose().x(), follower.getPose().y(), Math.toDegrees(follower.getPose().heading()));
            telemetry.update();
        }
        if (isStopRequested()) return;

        // 7. Start the route
        scheduler.schedule(route);

        // 8. Main loop
        while (opModeIsActive()) {
            scheduler.run();
            telemetry.addData("Path", follower.getCurrentPathName());
            telemetry.addData("Route", route.scheduled() ? "running" : "done");
            telemetry.addData("Last shot", useLut() ? String.format("LUT: %.1f in -> %.0f", lastShotDistanceIn, lastShotSpeed) : "fixed speed");
            telemetry.addData("Pose", "X: %.1f  Y: %.1f  H: %.1f",
                    follower.getPose().x(), follower.getPose().y(), Math.toDegrees(follower.getPose().heading()));
            telemetry.update();
        }

        // 9. Cleanup
        follower.stop();
    }

    // ---------------------------------------------------------------------------------------------
    // HELPERS for building routes
    // ---------------------------------------------------------------------------------------------

    /** The steps of a route, one after another. Each step gets the universal timeout. */
    protected Command sequential(Command... steps) {
        if (STEP_TIMEOUT_MS <= 0) {
            return new SequentialCommandGroup(steps);
        }
        Command[] timed = new Command[steps.length];
        for (int i = 0; i < steps.length; i++) {
            timed[i] = steps[i].withTimeout(stepTimeoutMs(steps[i]));
        }
        return new SequentialCommandGroup(timed);
    }

    /** Gives one step its own (longer) timeout, for example withStepTimeout(move(park()), 4500). */
    protected Command withStepTimeout(Command step, double timeoutMs) {
        expectedStepMs.put(step, timeoutMs);
        return step;
    }

    // How long each step needs at most, for steps that need more than STEP_TIMEOUT_MS
    private final java.util.Map<Command, Double> expectedStepMs = new java.util.IdentityHashMap<>();

    // The timeout a step gets: STEP_TIMEOUT_MS, or longer if the step needs it
    private double stepTimeoutMs(Command step) {
        Double expected = expectedStepMs.get(step);
        return Math.max(STEP_TIMEOUT_MS, expected != null ? expected : 0);
    }

    // Path names, so telemetry can show which path is running. Wrap each path method's path in named("...", ...).
    private final java.util.Map<Path, String> pathNames = new java.util.IdentityHashMap<>();

    protected Path named(String name, Path path) {
        pathNames.put(path, name);
        return path;
    }

    // Drives one path (used by move()). Kept with this signature so the Pedro visualizer's export works as written.
    protected Command follow(FollowerSubsystem follower, Path path) {
        String name = pathNames.get(path);
        return new FollowPathCommand(follower, path, name != null ? name : "path");
    }

    /** What the flower arm does during a move() */
    protected enum ArmMove { NONE, UP, DOWN }

    // A path move that can also run the intake and/or move the flower arm while it drives. Everything after the
    // path is optional (Java has no optional arguments, so each combination is its own short version):
    //   move(park())                           plain move
    //   move(garden(), 1250)                   intake runs for 1250 ms
    //   move(park(), ArmMove.UP)               arm goes up (waits ARM_UP_MS)
    //   move(park(), ArmMove.UP, 400)          arm goes up (waits 400 ms)
    //   move(flower1(), 4000, ArmMove.DOWN)    intake and arm (add a 4th number for the arm's wait time)
    protected Command move(Path path) {
        return move(path, 0, ArmMove.NONE, 0);
    }

    protected Command move(Path path, double intakeMs) {
        return move(path, intakeMs, ArmMove.NONE, 0);
    }

    protected Command move(Path path, ArmMove arm) {
        return move(path, 0, arm, 0);
    }

    protected Command move(Path path, ArmMove arm, double armMs) {
        return move(path, 0, arm, armMs);
    }

    protected Command move(Path path, double intakeMs, ArmMove arm) {
        return move(path, intakeMs, arm, 0);
    }

    // The one that does the work: the path, plus the intake and arm running at the same time
    protected Command move(Path path, double intakeMs, ArmMove arm, double armMs) {
        java.util.ArrayList<Command> parts = new java.util.ArrayList<>();
        parts.add(follow(follower, path));
        double armTime = 0;
        if (arm == ArmMove.UP) armTime = armMs > 0 ? armMs : ARM_UP_MS;
        if (arm == ArmMove.DOWN) armTime = armMs > 0 ? armMs : ARM_DOWN_MS;
        if (intakeMs > 0) parts.add(new RunIntakeCommand(intake, intakeMs));
        if (arm != ArmMove.NONE) parts.add(new FlowerArmCommand(intake, arm == ArmMove.UP, armTime));
        Command step = new ParallelCommandGroup(parts.toArray(new Command[0]));
        // A long intake/arm time gets that time (plus a margin) as its timeout instead of STEP_TIMEOUT_MS
        return withStepTimeout(step, Math.max(intakeMs, armTime) + STEP_MARGIN_MS);
    }

    // Lowers the flower arm and waits ARM_DOWN_MS, so the next move starts with the arm already down
    protected Command armDown() {
        return new FlowerArmCommand(intake, false, ARM_DOWN_MS);
    }

    // Raises the flower arm and waits ARM_UP_MS
    protected Command armUp() {
        return new FlowerArmCommand(intake, true, ARM_UP_MS);
    }

    // Shots. All numbers are optional, and 0 means "use the default":
    //   timeoutMs: if the flywheel isn't at speed after this long, it feeds anyway. 0 = SHOT_SPINUP_TIMEOUT_MS
    //              while the universal timeout is on (otherwise wait as long as it takes).
    //   feedMs: how long it feeds once the gate is open. 0 = SHOT_FEED_MS.
    //   shootCommand()                  default speed, timeout and feed time
    //   shootCommand(3000, 2000)        custom timeout and feed time
    //   shootCommand(1100, 0, 0)        custom speed for this one shot (skips the LUT)
    // Speed: with the LUT on (useLut()), shots without a custom speed use the LUT speed for the robot's distance to
    // the cell, worked out when the shot starts (same cell and table as teleop). With it off they use pollenSpeed().
    protected Command shootCommand() {
        return shootCommand(0, 0);
    }

    protected Command shootCommand(double timeoutMs, double feedMs) {
        if (useLut()) return makeShot(this::lutSpeed, timeoutMs, feedMs);
        double fixed = pollenSpeed();
        return makeShot(() -> fixed, timeoutMs, feedMs);
    }

    protected Command shootCommand(double pollenSpeed, double timeoutMs, double feedMs) {
        return makeShot(() -> pollenSpeed, timeoutMs, feedMs);
    }

    private Command makeShot(java.util.function.DoubleSupplier speed, double timeoutMs, double feedMs) {
        double feed = feedMs > 0 ? feedMs : SHOT_FEED_MS;
        double spinUp = timeoutMs > 0 ? timeoutMs : (STEP_TIMEOUT_MS > 0 ? SHOT_SPINUP_TIMEOUT_MS : 0);
        return withStepTimeout(new ShootCommand(shooter, intake, speed, spinUp, feed), spinUp + feed);
    }

    // LUT speed from where the robot is right now: picks the cell by robot Y like teleop does, then the distance
    // to it. Y >= 71 is blue's right cell / red's left cell (the same side of the hive, so the blue right table).
    protected double lutSpeed() {
        double x = follower.getPose().x();
        double y = follower.getPose().y();
        Pose2D cell = (y >= 71) ? FieldConstants.BLUE_GOAL_CASE_ONE : FieldConstants.BLUE_GOAL_CASE_TWO;   // right cell : left cell
        double distanceIn = Math.hypot(cell.getX(FieldConstants.DISTANCE_UNIT) - x, cell.getY(FieldConstants.DISTANCE_UNIT) - y);
        lastShotDistanceIn = distanceIn;
        lastShotSpeed = shooter.distanceToSpeed(distanceIn, y >= 71);
        return lastShotSpeed;
    }

    // Last LUT shot, shown on telemetry so you can check it
    protected double lastShotDistanceIn = Double.NaN;
    protected double lastShotSpeed = Double.NaN;
}
