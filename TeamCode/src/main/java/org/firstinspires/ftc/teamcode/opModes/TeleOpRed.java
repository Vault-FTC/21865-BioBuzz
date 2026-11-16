package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LaptopInput;
import org.firstinspires.ftc.teamcode.subsystems.PanelsSupport;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpRed extends LinearOpMode {
    Drivebase drivebase;
    Intake intake;
    Shooter shooter;
    // Driver Station and Panels (laptop) gamepads combined: read these instead of gamepad1 / gamepad2
    LaptopInput input = new LaptopInput();
    Gamepad pad1;
    Gamepad pad2;
    double errorDeg;
    double rx;
    double cellPosition; // this tells us what position the hive is in. If 1, the upward cell is on the right when standing in the blue driver station. if zero, that side is down.
    // Where this teleop's robot is placed at the start. Used when no auto has run, and by the gamepad2 Share reset.
    // Kept per teleop so Red and Blue can use different poses if auto-aim needs them to.
    static final double AIM_TOLERANCE_DEG = 3;
    boolean aimed;
    Pose2D teleopStartPose = FieldConstants.RED_TELEOP_START_POSE;
    Pose2D RedGoalCaseOne = FieldConstants.RED_GOAL_CASE_ONE;
    Pose2D RedGoalCaseTwo = FieldConstants.RED_GOAL_CASE_TWO;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = PanelsSupport.joined(telemetry);   // also show telemetry on Panels
        input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, gamepad1);

        double pollenTargetSpeed = Shooter.POLLEN_SPEED;   // defaults live in Shooter.java
        double pollenSpeedAdjustment = 0;
        cellPosition = 1; //cell starts with blue alliance cell up, red alliance cell down.

        // Use the shared field frame (inches, origin bottom-left, center (70, 70)); see FieldConstants.
        // If autonomous has run, continue from the last pose it saved. Teleop keeps updating that pose while
        // it runs, so restarting teleop also resumes from where the robot actually is. Otherwise start at
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
        double stickUpHeadingRad = Math.toRadians(FieldConstants.RED_STICK_UP_HEADING_DEG);

        // Init loop: shows which pose was loaded. Robot not where the last auto ended? Press gamepad2 Share.
        while (!isStarted() && !isStopRequested()) {
            input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
            drivebase.update();
            if (pad2.shareWasPressed()) {
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
            input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
            drivebase.update();
            PoseStorage.currentPose = drivebase.getPosition();   // keep the handoff pose current for a teleop restart

            // Options: re-zero the heading with the robot placed in the teleop starting orientation.
            if (pad1.optionsWasPressed() | pad2.optionsWasPressed()) {
                drivebase.resetHeading(teleopStartPose.getHeading(AngleUnit.DEGREES));
            }
            // Gamepad2 Share: the robot is at this teleop's normal starting spot (not where auto ended), so
            // reset the full pose to it.
            if (pad2.shareWasPressed()) {
                drivebase.setCurrentPose(teleopStartPose);
            }

            double y = pad1.left_stick_y;
            double x = -pad1.left_stick_x * 1.1;
            rx = pad1.right_stick_x;

            // DRIVER ONE
            doIntake();

            // Shooters are always spinning with the gates closed. Right bumper: auto-aim, then open the
            // gates and feed once both flywheels are at speed. Right trigger: same shot, no auto-aim.
            if (pad1.right_bumper) {
                autoAim();   // adds a correction to rx
                if (aimed) {
                    shooter.setState(Shooter.CaseModes.SHOOT);
                } else {
                    shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
                }
            } else if (pad1.right_trigger_pressed) {
                shooter.setState(Shooter.CaseModes.SHOOT);
            } else {
                shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
            }

            drivebase.drive(-y, -x, rx, stickUpHeadingRad);

            // DRIVER TWO
            if (pad2.squareWasPressed()) {
                cellPosition = (cellPosition == 1) ? 0 : 1;
            }
            if (pad2.dpadUpWasPressed()) {
                pollenSpeedAdjustment += 25;
            }
            if (pad2.dpadDownWasPressed()) {
                pollenSpeedAdjustment -= 25;
            }
            if (pad2.right_bumper) {
                double positionInCycle = System.currentTimeMillis() % 1000;
                intake.setState(positionInCycle < 100 ? Intake.CaseModes.ON : Intake.CaseModes.OFF);
            }

            shooter.setManualGateOpen(pad1.triangle || pad2.triangle);   // hold Triangle to force the gates open

            shooter.setTargetSpeed(pollenTargetSpeed + pollenSpeedAdjustment);
            shooter.update();   // in SHOOT mode this overrides the intake state
            intake.update();

            telemetry.addData("Shooter Target (pollen)", pollenTargetSpeed + pollenSpeedAdjustment);
            telemetry.addData("Pollen Speed", shooter.getPollenShooterVelocity());
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.addData("Laptop gamepad (Panels)", input.status());
            telemetry.addData("Cell Position", cellPosition);
            telemetry.addData("Angle to goal", errorDeg = (Drivebase.angleToGoal(drivebase.getPosition(), RedGoalCaseOne)) * (180 / Math.PI));
            telemetry.update();
        }
    }

    public void doIntake() {
        if (pad2.crossWasPressed()) {
            intake.toggleFlowerArm();
        }

        if (pad1.left_bumper) {
            intake.setState(Intake.CaseModes.ON);
        } else if (pad1.square) {
            intake.setState(Intake.CaseModes.REVERSE);
            shooter.openGate();
        } else {
            intake.setState(Intake.CaseModes.OFF);
        }
    }

    public void autoAim() {
        Pose2D goal = (cellPosition == 1) ? RedGoalCaseOne : RedGoalCaseTwo;
        errorDeg = (Drivebase.angleToGoal(drivebase.getPosition(), goal)) * (180 / Math.PI);
        aimed = Math.abs(errorDeg) < AIM_TOLERANCE_DEG;
        rx = rx + errorDeg * -0.015;
    }
}
