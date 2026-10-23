package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tunes the pollen and nectar shooters at the same time. Both motors always run at their own target velocity;
 * Options selects which one the Cross/Square and D-Pad adjustments apply to.
 * Left bumper runs the intake, left trigger runs it in reverse, and Share toggles the shooter gates open/closed.
 */
@TeleOp
public class ShooterPIDTuner extends OpMode {
    /** Tuning state for one shooter motor. */
    private static class Flywheel {
        final String name;
        final DcMotorEx motor;
        double targetVelocity = 0;
        double P = 0.0;
        double F = 13.453;

        final DcMotorEx.Direction direction;

        Flywheel(String name, DcMotorEx motor, DcMotorEx.Direction direction) {
            this.name = name;
            this.motor = motor;
            this.direction = direction;
        }

        void apply() {
            motor.setDirection(direction);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
            motor.setVelocity(targetVelocity);
        }
    }

    // Gate positions copied from Shooter.openGate()/closeGate(); keep them in sync if those change.
    static final double POLLEN_GATE_CLOSED = 0.7;
    static final double POLLEN_GATE_OPEN = 1.0;
    static final double NECTAR_GATE_CLOSED = 1.0;
    static final double NECTAR_GATE_OPEN = 0.6;

    public DcMotorEx intake;
    Servo pollenGate;
    Servo nectarGate;
    boolean gatesOpen = false;
    Flywheel pollen;
    Flywheel nectar;
    Flywheel selected;

    double[] pidStepSizes = {50, 10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int pidStepIndex = 0;

    double[] velocityStepSizes = {10.0, 50.0, 100.0, 500.0, 950.0, 1000.0, 1300.0, 1310.0};
    int velocityStepIndex = 0;

    @Override
    public void init() {
        pollen = new Flywheel("Pollen", hardwareMap.get(DcMotorEx.class, "pollenShooter"), DcMotorEx.Direction.REVERSE);
        nectar = new Flywheel("Nectar", hardwareMap.get(DcMotorEx.class, "nectarShooter"), DcMotorEx.Direction.FORWARD);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        nectarGate = hardwareMap.get(Servo.class, "nectarGate");
        selected = pollen;

        for (Flywheel f : new Flywheel[]{pollen, nectar}) {
            f.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            f.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            f.apply();
        }
        setGates(false);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // Options: choose which shooter the adjustments below apply to
        if (gamepad1.optionsWasPressed()) {
            selected = (selected == pollen) ? nectar : pollen;
        }

        // Cycle which velocity step size is active
        if (gamepad1.triangleWasPressed()) {
            velocityStepIndex = (velocityStepIndex + 1) % velocityStepSizes.length;
        }
        // Adjust the selected shooter's target velocity by the active step size
        if (gamepad1.crossWasPressed()) {
            selected.targetVelocity += velocityStepSizes[velocityStepIndex];
        }
        if (gamepad1.squareWasPressed()) {
            selected.targetVelocity -= velocityStepSizes[velocityStepIndex];
        }

        // Cycle which P/F step size is active
        if (gamepad1.circleWasPressed()) {
            pidStepIndex = (pidStepIndex + 1) % pidStepSizes.length;
        }
        if (gamepad1.dpadRightWasPressed()) {
            selected.F += pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            selected.F -= pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            selected.P -= pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            selected.P += pidStepSizes[pidStepIndex];
        }

        // Share: toggle the gates open/closed (they stay where you leave them)
        if (gamepad1.shareWasPressed()) {
            setGates(!gatesOpen);
        }

        // Left bumper: intake in (feed a game piece so you're tuning under realistic load).
        // Left trigger: outtake.
        if (gamepad1.left_bumper) {
            intake.setPower(0.5);
        } else if (gamepad1.left_trigger_pressed) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        pollen.apply();
        nectar.apply();

        telemetry.addData("Gates (Share to toggle)", gatesOpen ? "OPEN" : "closed");
        telemetry.addData("Selected (Options to switch)", selected.name);
        telemetry.addData("Velocity Step Size", "%.0f (Triangle to cycle, Cross +/Square -)", velocityStepSizes[velocityStepIndex]);
        telemetry.addData("PID Step Size", "%.4f (Circle button)", pidStepSizes[pidStepIndex]);
        for (Flywheel f : new Flywheel[]{pollen, nectar}) {
            double curVelocity = f.motor.getVelocity();
            telemetry.addLine("----------- " + f.name + " -----------");
            telemetry.addData(f.name + " Target Velocity", "%.0f", f.targetVelocity);
            telemetry.addData(f.name + " Current Velocity", "%.2f", curVelocity);
            telemetry.addData(f.name + " Error", "%.2f", f.targetVelocity - curVelocity);
            telemetry.addData(f.name + " P (D-Pad U/D)", "%.4f", f.P);
            telemetry.addData(f.name + " F (D-Pad L/R)", "%.4f", f.F);
        }
    }

    private void setGates(boolean open) {
        gatesOpen = open;
        pollenGate.setPosition(open ? POLLEN_GATE_OPEN : POLLEN_GATE_CLOSED);
        nectarGate.setPosition(open ? NECTAR_GATE_OPEN : NECTAR_GATE_CLOSED);
    }
}
