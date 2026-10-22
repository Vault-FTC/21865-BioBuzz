package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpBlue extends LinearOpMode {
    Drivebase drivebase;
    Intake intake;
    Shooter shooter;
    double errorDeg;
    double rx;
    double cellPosition; // this tells us what position the hive is in. If 1, the upward cell is on the right when standing in the blue driver station. if zero, that side is down.
    // Where this teleop's robot is placed at the start. Used when no auto has run, and by the gamepad2 Share reset.
    // Kept per teleop so Red and Blue can use different poses if auto-aim needs them to.
    Pose2D teleopStartPose = FieldConstants.BLUE_TELEOP_START_POSE;
    Pose2D BlueGoalCaseOne = FieldConstants.BLUE_GOAL_CASE_ONE;
    Pose2D BlueGoalCaseTwo = FieldConstants.BLUE_GOAL_CASE_TWO;

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, gamepad1);

        double pollenTargetSpeed = Shooter.POLLEN_SPEED;   // defaults live in Shooter.java
        double nectarTargetSpeed = Shooter.NECTAR_SPEED;
        double pollenSpeedAdjustment = 0;
        double nectarSpeedAdjustment = 0;
        boolean pollenSelected = true;   // which shooter gamepad2's d-pad adjusts
        cellPosition = 1; //cell starts with blue alliance cell up, red alliance cell down.

        // Use the shared field frame (inches, origin bottom-left, center (70, 70)); see FieldConstants.
        // If autonomous has run, continue from the last pose it saved. Teleop keeps updating that pose while
        // it runs, so restarting teleop also resumes from where the robot actually is. Otherwise, start at
        // teleopStartPose (the same spot and heading as this alliance's autonomous start). Gamepad2 Share
        // resets to teleopStartPose when the robot has been re-placed.
        String poseSource;
        if (PoseStorage.currentPose != null) {
            drivebase.setCurrentPose(PoseStorage.currentPose);
            poseSource = "saved auto pose";
        } else {
            drivebase.getPinpoint().resetPosAndIMU();
            sleep(300);   // let the Pinpoint finish its IMU reset before we write a pose to it
            drivebase.setCurrentPose(teleopStartPose);
            poseSource = "teleop start pose (no auto saved one)";
        }
        // NOTE: the SHOOTER end is the robot's front. The Pinpoint heading and "forward" in the drive code both
        // refer to that end.
        // Field direction the drive stick pushes toward.
        double stickUpHeadingRad = Math.toRadians(FieldConstants.BLUE_STICK_UP_HEADING_DEG);

        // Init loop: shows which pose was loaded. Robot not where the last auto ended? Press gamepad2 Share.
        while (!isStarted() && !isStopRequested()) {
            drivebase.update();
            if (gamepad2.shareWasPressed()) {
                drivebase.setCurrentPose(teleopStartPose);
                poseSource = "teleop start pose (Share pressed)";
            }
            telemetry.addData("Pose source", poseSource);
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drivebase.update();
            PoseStorage.currentPose = drivebase.getPosition();   // keep the handoff pose current for a teleop restart

            // Options: re-zero the heading with the robot placed in the teleop starting orientation.
            if (gamepad1.optionsWasPressed()) {
                drivebase.resetHeading(teleopStartPose.getHeading(AngleUnit.DEGREES));
            }
            // Gamepad2 Share: the robot is at this teleop's normal starting spot (not where auto ended), so
            // reset the full pose to it.
            if (gamepad2.shareWasPressed()) {
                drivebase.setCurrentPose(teleopStartPose);
            }

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;

            // DRIVER ONE
            doIntake();

            // Shooters are always spinning with the gates closed. Right bumper: auto-aim, then open the
            // gates and feed once both flywheels are at speed. Right trigger: same shot, no auto-aim.
            if (gamepad1.right_bumper) {
                autoAim();   // adds a correction to rx
                shooter.setState(Shooter.CaseModes.SHOOT);
            } else if (gamepad1.right_trigger_pressed) {
                shooter.setState(Shooter.CaseModes.SHOOT);
            } else {
                shooter.setState(Shooter.CaseModes.OFF); // Should be shoot_gate_closed
            }

            drivebase.drive(-y, -x, rx, stickUpHeadingRad);

            // DRIVER TWO
            if (gamepad2.squareWasPressed()) {
                cellPosition = (cellPosition == 1) ? 0 : 1;
            }
            if (gamepad2.circleWasPressed()) {
                pollenSelected = !pollenSelected;
            }
            if (gamepad2.dpadUpWasPressed()) {
                if (pollenSelected) pollenSpeedAdjustment += 25;
                else nectarSpeedAdjustment += 25;
            }
            if (gamepad2.dpadDownWasPressed()) {
                if (pollenSelected) pollenSpeedAdjustment -= 25;
                else nectarSpeedAdjustment -= 25;
            }
            if (gamepad2.right_bumper) {
                double positionInCycle = System.currentTimeMillis() % 1000;
                intake.setState(positionInCycle < 100 ? Intake.CaseModes.ON : Intake.CaseModes.OFF);
            }
            shooter.setManualGateOpen(gamepad1.triangle || gamepad2.triangle);   // hold Triangle to force the gates open

            shooter.setTargetSpeeds(pollenTargetSpeed + pollenSpeedAdjustment, nectarTargetSpeed + nectarSpeedAdjustment);
            shooter.update();   // in SHOOT mode this overrides the intake state
            intake.update();

            telemetry.addData("Selected Shooter (gamepad2 Circle)", pollenSelected ? "Pollen" : "Nectar");
            telemetry.addData("Shooter Target (pollen)", pollenTargetSpeed + pollenSpeedAdjustment);
            telemetry.addData("Shooter Target (nectar)", nectarTargetSpeed + nectarSpeedAdjustment);
            telemetry.addData("Pollen Speed", shooter.getPollenShooterVelocity());
            telemetry.addData("Nectar Speed", shooter.getNectarShooterVelocity());
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.addData("Cell Position", cellPosition);
            telemetry.addData("Angle to goal", errorDeg = (Drivebase.angleToGoal(drivebase.getPosition(), BlueGoalCaseOne)) * (180 / Math.PI));
            telemetry.update();
        }
    }

    public void doIntake() {
        if (gamepad1.left_bumper) {
            intake.setState(Intake.CaseModes.ON);
        } else if (gamepad1.square) {
            intake.setState(Intake.CaseModes.REVERSE);
        } else if (gamepad1.leftTriggerWasPressed()) {
            intake.setState(Intake.CaseModes.INTAKE_FROM_FLOWER);
        } else {
            intake.setState(Intake.CaseModes.OFF);
        }
    }

    public void autoAim() {
        Pose2D goal = (cellPosition == 1) ? BlueGoalCaseOne : BlueGoalCaseTwo;
        errorDeg = (Drivebase.angleToGoal(drivebase.getPosition(), goal)) * (180 / Math.PI);
        rx = rx + errorDeg * -0.015;
    }
}
