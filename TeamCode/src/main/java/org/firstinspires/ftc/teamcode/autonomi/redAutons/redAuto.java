package org.firstinspires.ftc.teamcode.autonomi.redAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.autonomi.Location;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Shoots from the starting spot with both shooters, then moves toward -X (field left) to leave.
 * All positions are in the shared field frame (inches, field center at (70, 70)); see FieldConstants.
 */
@Autonomous
public class redAuto extends LinearOpMode {
    static final double POLLEN_SPEED = 1300;
    static final double NECTAR_SPEED = 1250;

    static final double SPIN_UP_TIMEOUT_S = 3.0;   // if the flywheels never report full speed, feed anyway after this
    static final double FEED_TIME_S = 4.0;         // how long to feed once the gates open
    static final double DRIVE_BASE_TIMEOUT_S = 2.0;      // each drive gets this long...
    static final double DRIVE_SECONDS_PER_INCH = 0.15;   // ...plus this much per inch to cover (a guess; tune it)

    // How far to drive left (toward -X, decreasing X) for the first move
    static final double LEAVE_LEFT_IN = 24;

    Drivebase drivebase;
    Intake intake;
    Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, drivebase, intake, gamepad1);
        shooter.setRumbleEnabled(false);   // no controller rumble during autonomous

        // Put the odometry in the shared field frame.
        drivebase.getPinpoint().resetPosAndIMU();
        sleep(300);   // let the Pinpoint finish its IMU reset before we write a pose to it
        Pose2D start = new Pose2D(DistanceUnit.INCH, 59, 8.5, AngleUnit.DEGREES, 90);
        drivebase.setCurrentPose(start);
        PoseStorage.currentPose = start;

        double startHeadingDeg = start.getHeading(AngleUnit.DEGREES);
        Location shootSpot = new Location(
                start.getX(FieldConstants.DISTANCE_UNIT),
                start.getY(FieldConstants.DISTANCE_UNIT),
                startHeadingDeg);
        // Leave by decreasing X while keeping the same Y and heading.
        Location moveLeft = new Location(
                24,
                8,
                startHeadingDeg);
        Location driveForwards = new Location(
                32,
                99,
                90);
        Location park = new Location(
                10,
                99,
                90);

        shooter.setTargetSpeeds(POLLEN_SPEED, NECTAR_SPEED);

        // Show the live pose during init so you can confirm the heading before pressing start.
        while (!isStarted() && !isStopRequested()) {
            drivebase.update();
            telemetry.addLine("Place the robot at the start pose, then press start.");
            telemetry.addData("Expected start", "X: %.1f  Y: %.1f  H: %.0f",
                    start.getX(FieldConstants.DISTANCE_UNIT), start.getY(FieldConstants.DISTANCE_UNIT),
                    start.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        // Flywheels spin from the very start with the gates closed, and keep spinning for the whole auto.
        shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);

        shoot();
        driveTo("moveLeft", moveLeft);
        driveTo("driveForwards", driveForwards);
        driveTo("park", park);

        // Nothing here ends the auto: keep the flywheels spinning and hold still until the driver station's
        // 30 second timer stops the opmode.
        while (opModeIsActive()) {
            drivebase.update();
            drivebase.drive(0, 0, 0);
            updateSubsystems();
            PoseStorage.currentPose = drivebase.getPosition();
            telemetry.addData("Step", "Done, holding");
            telemetry.update();
        }
        PoseStorage.currentPose = drivebase.getPosition();
    }

    /**
     * Opens the gates and feeds once both flywheels are at speed. If they haven't reached speed after
     * SPIN_UP_TIMEOUT_S it feeds anyway. Afterward the gates close again and the flywheels keep spinning.
     */
    void shoot() {
        shooter.setState(Shooter.CaseModes.SHOOT);
        ElapsedTime spinUpTimer = new ElapsedTime();
        ElapsedTime feedTimer = new ElapsedTime();
        boolean feeding = false;
        while (opModeIsActive()) {
            drivebase.update();
            boolean atSpeed = shooter.getPollenShooterVelocity() >= POLLEN_SPEED
                    && shooter.getNectarShooterVelocity() >= NECTAR_SPEED;
            if (!feeding && (atSpeed || spinUpTimer.seconds() >= SPIN_UP_TIMEOUT_S)) {
                feeding = true;
                feedTimer.reset();
            }
            if (feeding && feedTimer.seconds() >= FEED_TIME_S) break;

            shooter.setManualGateOpen(feeding);   // forces the gates open if we timed out waiting for speed
            shooter.update();
            if (feeding) intake.setState(Intake.CaseModes.ON);   // SHOOT only runs the intake at half speed until it's at speed
            intake.update();
            telemetry.addData("Step", feeding ? "Feeding" : "Spinning up");
            telemetry.addData("Pollen Speed", shooter.getPollenShooterVelocity());
            telemetry.addData("Nectar Speed", shooter.getNectarShooterVelocity());
            telemetry.update();
        }
        shooter.setManualGateOpen(false);
        shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
        intake.setState(Intake.CaseModes.OFF);
        updateSubsystems();
    }

    /** Keeps the shooter (flywheels and gates) and intake running; call once per loop pass. */
    void updateSubsystems() {
        shooter.update();
        intake.update();
    }

    /**
     * Drives to a field target with Drivebase.driveToPosition until it arrives or times out. The timeout grows
     * with the distance, and a one-line result (arrived or timed out, and the distance before and after) is added
     * to the Driver Station log so you can tell a short timeout from a wrong-way drive: if the distance after is
     * bigger than the distance before, the robot is driving away from the target.
     */
    void driveTo(String label, Location target) {
        drivebase.update();
        double startDistance = distanceTo(target);
        double timeoutS = DRIVE_BASE_TIMEOUT_S + startDistance * DRIVE_SECONDS_PER_INCH;
        ElapsedTime timer = new ElapsedTime();
        boolean arrived = false;
        while (opModeIsActive() && timer.seconds() < timeoutS) {
            drivebase.update();
            updateSubsystems();
            if (drivebase.isAtPosition(target, 1, 2)) {
                arrived = true;
                break;
            }
            drivebase.driveToPosition(target, telemetry);
            telemetry.addData("Step", "%s (%.1f / %.1f s)", label, timer.seconds(), timeoutS);
            addPoseTelemetry();
        }
        drivebase.drive(0, 0, 0);
        drivebase.update();
        Pose2D pose = drivebase.getPosition();
        telemetry.log().add(String.format("%s: %s in %.1fs, dist %.1f -> %.1f in, pose X %.1f Y %.1f H %.0f",
                label, arrived ? "arrived" : "TIMED OUT", timer.seconds(), startDistance, distanceTo(target),
                pose.getX(FieldConstants.DISTANCE_UNIT), pose.getY(FieldConstants.DISTANCE_UNIT),
                pose.getHeading(AngleUnit.DEGREES)));
    }

    /** Straight-line distance from the robot to a target, in field inches. */
    double distanceTo(Location target) {
        Pose2D pose = drivebase.getPosition();
        return Math.hypot(target.x - pose.getX(FieldConstants.DISTANCE_UNIT),
                target.y - pose.getY(FieldConstants.DISTANCE_UNIT));
    }

    /** Holds the robot on a spot, cutting power once it is close enough so it doesn't jitter. */
    void holdOrDrive(Location target) {
        if (drivebase.isAtPosition(target, 1, 2)) {
            drivebase.drive(0, 0, 0);
        } else {
            drivebase.driveToPosition(target, telemetry);
        }
    }

    void addPoseTelemetry() {
        PoseStorage.currentPose = drivebase.getPosition();
        telemetry.addData("Position", drivebase.getPositionTelemetry());
        telemetry.update();
    }
}
