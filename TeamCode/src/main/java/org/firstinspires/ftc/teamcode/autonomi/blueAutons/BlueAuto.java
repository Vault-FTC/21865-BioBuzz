package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

// =================================================================================================
// IMPORTS
// Every class you use in this file has to be imported here, one line per class. In Android Studio, type the
// class name and press Alt+Enter to import it automatically. Unused imports are harmless.
// =================================================================================================
import com.pedropathing.api.Paths;
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
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * BLUE AUTO (skeleton). The numbered steps inside runOpMode() are the order everything happens in. Fill in the
 * "TODO" spots.
 *
 * <p>THE TWO KINDS OF OPMODE
 * <ul>
 *   <li><b>LinearOpMode</b> (what this file uses): you write ONE method, {@code runOpMode()}, and it runs top
 *       to bottom exactly once. You write your own {@code while} loops. Use {@code waitForStart()} to pause
 *       until the driver presses start, {@code opModeIsActive()} as the loop condition (it turns false when
 *       the driver presses stop or time runs out), and {@code sleep(ms)} to wait. Good for autonomous and for
 *       simple teleops.</li>
 *   <li><b>OpMode</b> (the "iterative" kind): you write several methods and the robot controller calls them for
 *       you: {@code init()} once on init, {@code init_loop()} repeatedly until start, {@code start()} once,
 *       {@code loop()} over and over, {@code stop()} once. You must NOT use {@code sleep()} or your own
 *       {@code while} loops, because they would freeze {@code loop()}. Variables that must survive between
 *       calls have to be class-wide. Good for code that is just "run this every cycle".</li>
 * </ul>
 * Both use {@code @Autonomous} or {@code @TeleOp} above the class so they show up on the Driver Station.
 *
 * <p>THE COMMAND SYSTEM (how the steps below work together)
 * You build the whole autonomous as commands BEFORE it starts, hand the finished route to the scheduler, and
 * then call {@code scheduler.run()} in a loop. Each call updates every subsystem (follower, shooter, intake)
 * and advances the commands. Because of that, never call a subsystem's {@code update()} yourself in an auto.
 */
@Autonomous(name = "Blue Auto", group = "Blue")
public class BlueAuto extends LinearOpMode {

    // =============================================================================================
    // CLASS-WIDE VARIABLES (fields)
    // These are declared inside the class but OUTSIDE every method. They live as long as the opmode and every
    // method in this class can see them.
    //   Put here: fixed numbers you want to find and edit easily (the "static final" constants), and anything
    //   that more than one method needs (here: the subsystems, so helper methods like shootCommand() can use
    //   them).
    //   "static final" = a constant that never changes. A plain field (no static final) can be changed later.
    //
    // WARNING: declare a subsystem here, but create it INSIDE runOpMode() (step 2). hardwareMap does not exist
    // yet when the class is first built, so "private Intake intake = new Intake(hardwareMap);" up here crashes.
    // =============================================================================================

    // Blue start pose from the Pedro visualizer. Field inches and degrees. 270 = facing down the field (-Y).
    static final double START_X_IN = 82.5;
    static final double START_Y_IN = 133;
    static final double START_HEADING_DEG = 270;

    // Shooter target speeds. TODO: set these once the shooter PID is tuned (0 keeps the flywheels off).
    static final double POLLEN_SPEED = 0;
    static final double NECTAR_SPEED = 0;

    private FollowerSubsystem follower;
    private Intake intake;
    private Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        // =========================================================================================
        // FUNCTION-WIDE VARIABLES (locals)
        // Variables declared inside runOpMode() only exist inside runOpMode() and disappear when it ends.
        //   Put here: things only this method uses, like the poses, the paths and the finished route.
        // =========================================================================================

        // 1. SCHEDULER RESET. Always first. It clears the commands and subsystems left over from the last run.
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.clearRegistry();

        // 2. HARDWARE / SUBSYSTEMS. Creating a subsystem looks up its motors and servos in the hardware map and
        //    registers it with the scheduler. Create Intake before Shooter (the shooter uses the intake).
        //    To add another subsystem: import it above, declare it as a class-wide field, create it here.
        follower = new FollowerSubsystem(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, gamepad1);

        // 3. SUBSYSTEM SETUP. Settings that stay the same for the whole auto.
        shooter.setRumbleEnabled(false);                       // no controller rumble in auto
        shooter.setTargetSpeeds(POLLEN_SPEED, NECTAR_SPEED);   // stores the speeds; it does not spin the motors

        // 4. START POSE. Tell the follower where the robot is. The wait lets the Pinpoint finish resetting first.
        //    Pedro poses are inches and RADIANS.
        sleep(300);
        double startHeading = Math.toRadians(START_HEADING_DEG);
        Pose start = new Pose(START_X_IN, START_Y_IN, startHeading);
        follower.setPose(start);

        // 5. PATHS. One Path per move. A Path needs a heading rule:
        //      .constant(h)    keep heading h the whole way (a strafe)
        //      .linear(a, b)   turn from pose a's heading to pose b's heading while driving
        //      .tangent()      point along the direction of travel
        //    TODO: create your poses and paths here, for example:
        //      Pose spot = new Pose(40, 40, startHeading);
        //      Path toSpot = Paths.line(start, spot).constant(startHeading);

        // 6. THE ROUTE. Everything the robot does, in the order it happens:
        //    - move commands:       new FollowPathCommand(follower, toSpot)
        //    - subsystem commands:  new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT))
        //    - wait for something:  new DelayCommand(shooter::isShootReady)
        //    - several things at once: ParallelCommandGroup.getBuilder().add(a).add(b).build()
        //    The route is a SequentialCommandGroup: it runs its commands one at a time, top to bottom.
        //    TODO: replace the placeholder with your commands. (The group crashes if it is empty, which is why
        //    there is one do-nothing command in it.)
        Command route = new SequentialCommandGroup(
                new InstantCommand(() -> { /* TODO: first command */ })
        );

        // 7. INIT LOOP. Runs while you wait for the driver to press start. Calling scheduler.run() keeps the
        //    follower's pose live so you can read it. Put telemetry about the start state here.
        while (!isStarted() && !isStopRequested()) {
            scheduler.run();
            telemetry.addData("Pose", "X: %.1f  Y: %.1f  H: %.1f",
                    follower.getPose().x(), follower.getPose().y(), Math.toDegrees(follower.getPose().heading()));
            telemetry.update();
        }
        if (isStopRequested()) return;

        // 8. START. Hand the route to the scheduler.
        scheduler.schedule(route);

        // 9. MAIN LOOP. Runs until the driver station's timer or Stop ends the opmode. scheduler.run() does all
        //    the work. Nothing here ends the opmode early. Put per-loop telemetry here.
        while (opModeIsActive()) {
            scheduler.run();
            telemetry.addData("Route", route.scheduled() ? "running" : "done");
            telemetry.update();
        }

        // 10. CLEANUP. Runs once, after the loop ends.
        follower.stop();
    }

    // =============================================================================================
    // HELPER METHODS
    // Put your own methods below runOpMode(), as members of the class. Use them to avoid repeating the same
    // commands. A helper that builds commands has to be called from step 6 (before the route is scheduled),
    // not from the main loop.
    // =============================================================================================

    /**
     * EXAMPLE (not used yet): the commands for one shot. Call it inside the route, for example
     * {@code new SequentialCommandGroup(new FollowPathCommand(follower, toSpot), shootCommand())}.
     * TODO: add a timed wait for the feed time (uncomment commands/WaitCommand.java; it takes milliseconds)
     * between the DelayCommand and the last InstantCommand.
     */
    private Command shootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT)),
                new DelayCommand(shooter::isShootReady),
                new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED))
        );
    }
}
