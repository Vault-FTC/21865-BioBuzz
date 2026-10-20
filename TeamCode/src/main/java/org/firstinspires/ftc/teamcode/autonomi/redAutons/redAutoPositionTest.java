package org.firstinspires.ftc.teamcode.autonomi.redAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.autonomi.Location;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

/**
 * Odometry test with the same setup and waypoints as redAuto, but the motors never run.
 * Push the robot by hand and watch the telemetry. The wheels coast so the robot is easy to move.
 *
 * Each waypoint is marked reached when you get within redAuto's arrival tolerance
 * (1 inch and 2 degrees), and the next one is shown.
 * Gamepad 1: A skips to the next waypoint. B resets the pose to the start and restarts the route.
 */
@Autonomous(name = "redAuto Position Test", group = "Test")
public class redAutoPositionTest extends LinearOpMode {
    // Same arrival tolerance redAuto passes to isAtPosition.
    static final double TOLERANCE_IN = 1;
    static final double TOLERANCE_DEG = 2;

    Drivebase drivebase;

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Drivebase(hardwareMap);
        drivebase.setToCoastMode();
        drivebase.drive(0, 0, 0);

        // Same start pose as redAuto. Keep the two in sync if you change it.
        Pose2D start = new Pose2D(DistanceUnit.INCH, 59, 8.5, AngleUnit.DEGREES, 90);
        resetToStart(start);

        // Same waypoints as redAuto. Keep the two in sync if you change them.
        String[] names = {"moveLeft", "driveForwards", "park"};
        Location[] waypoints = {
                new Location(24, 8, start.getHeading(AngleUnit.DEGREES)),
                new Location(32, 99, 90),
                new Location(15, 99, 90),
        };
        int step = 0;

        // Show the pose during init too, so you can check it before pressing start.
        while (opModeInInit()) {
            drivebase.update();
            telemetry.addLine("Place the robot at the start pose, then press start.");
            addPose(start);
            telemetry.update();
        }

        while (opModeIsActive()) {
            drivebase.update();

            if (gamepad1.bWasPressed()) {
                resetToStart(start);
                step = 0;
            }
            if (gamepad1.aWasPressed() && step < waypoints.length) {
                step++;
            }
            if (step < waypoints.length
                    && drivebase.isAtPosition(waypoints[step], TOLERANCE_IN, TOLERANCE_DEG)) {
                step++;
            }

            addPose(start);
            if (step < waypoints.length) {
                Location target = waypoints[step];
                Pose2D pose = drivebase.getPosition();
                telemetry.addData("Next waypoint", "%d/%d %s", step + 1, waypoints.length, names[step]);
                telemetry.addData("Target", "X: %.1f  Y: %.1f  H: %.1f", target.x, target.y, target.headingDegrees);
                telemetry.addData("Still to go", "X: %+.1f  Y: %+.1f  H: %+.1f",
                        target.x - pose.getX(DistanceUnit.INCH),
                        target.y - pose.getY(DistanceUnit.INCH),
                        FieldConstants.normalizeDegrees(target.headingDegrees - pose.getHeading(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine("All waypoints reached.");
            }
            telemetry.addLine("A: skip waypoint   B: reset to start");
            telemetry.update();
        }
    }

    void resetToStart(Pose2D start) {
        drivebase.getPinpoint().resetPosAndIMU();
        sleep(300);   // let the Pinpoint finish its IMU reset before we write a pose to it
        drivebase.setCurrentPose(start);
    }

    void addPose(Pose2D start) {
        Pose2D pose = drivebase.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        telemetry.addData("Position", "X: %.1f in  Y: %.1f in  H: %.1f deg", x, y, pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Moved from start", "X: %+.1f  Y: %+.1f",
                x - start.getX(DistanceUnit.INCH), y - start.getY(DistanceUnit.INCH));
    }
}
