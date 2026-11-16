package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;

public class Shooter extends Subsystem {
    public enum CaseModes {
        OFF, SHOOT, SHOOT_GATE_CLOSED, REVERSE, AUTO_SHOOT
    }

    private final DcMotorEx pollenShooter;
    private final Servo pollenGate;
    private final Intake intake;
    // Default flywheel target speeds for teleop and autonomous. Change them here to change them everywhere.
    public static final double POLLEN_SPEED = 1150;
    // Gate servo positions. ShooterPIDTuner uses these too, so change them here only.
    public static final double POLLEN_GATE_CLOSED = 0.499;
    public static final double POLLEN_GATE_OPEN = 0.5;

    double pollenSpeed = POLLEN_SPEED;
    CaseModes currentMode = CaseModes.OFF;
    boolean manualGateOpen = false;
    boolean rumbleEnabled = true;   // teleop rumbles when the shooter is up to speed; autos turn this off
    boolean shootReady = false;   // latched once the flywheels reach speed in SHOOT; cleared when leaving SHOOT
    PIDFCoefficients pollenPIDF = new PIDFCoefficients(167.8, 0, 0, 12.421); // F from the 2026-11-16 tuner runs at 1350 (they gave 12.174 and 12.421; before that 12.824)
    Gamepad gamepad1; //what is this?

    public Shooter(HardwareMap hardwareMap, Intake intake, Gamepad gamepad) {
        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        this.intake = intake;
        pollenShooter.setDirection(DcMotorEx.Direction.REVERSE);
        pollenShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pollenPIDF);
        gamepad1 = gamepad; //what is this?
    }

    /** In autonomous the command scheduler drives this; teleop calls {@link #update()} directly. */
    @Override
    public void periodic() {
        update();
    }

    public void update() {
        boolean gateOpen = false;
        switch(currentMode){
            case OFF:
                pollenShooter.setVelocity(0);
                break;
            case SHOOT:
                pollenShooter.setVelocity(pollenSpeed);
                //intake.setState(Intake.CaseModes.HALF_SPEED);
                // Once both flywheels have reached speed, stay in the firing state until SHOOT is exited,
                // even if the speed dips as balls pass through.
                if (!shootReady && getPollenShooterVelocity() >= pollenSpeed) {
                    shootReady = true;
                }
                if (shootReady) {
                    gateOpen = true;
                    intake.setState(Intake.CaseModes.ON);
                    if (rumbleEnabled) gamepad1.rumble(1000);
                }
                break;
            case AUTO_SHOOT:
                pollenShooter.setVelocity(pollenSpeed);
                //intake.setState(Intake.CaseModes.HALF_SPEED);
                // Once both flywheels have reached speed, stay in the firing state until SHOOT is exited,
                // even if the speed dips as balls pass through.
                if (!shootReady && getPollenShooterVelocity() >= pollenSpeed) {
                    shootReady = true;
                }
                if (shootReady) {
                    gateOpen = true;
                    intake.setState(Intake.CaseModes.SIXTY_PERCENT_SPEED);
                    if (rumbleEnabled) gamepad1.rumble(1000);
                }
                break;
            case SHOOT_GATE_CLOSED:
                pollenShooter.setVelocity(pollenSpeed);
                // doesn't touch the intake, so the driver keeps manual intake control
                break;
            case REVERSE:
                gateOpen = true;
                intake.setState(Intake.CaseModes.REVERSE);
                pollenShooter.setVelocity(-900);
                break;
        }
        // The gates are written once per loop: open if the mode wants them open OR the driver is holding the manual override.
        if (gateOpen || manualGateOpen) {
            openGate();
        } else {
            closeGate();
        }
    }

    /** Turns the "up to speed" controller rumble on or off. On by default; autos turn this off. */
    public void setRumbleEnabled(boolean enabled) {
        rumbleEnabled = enabled;
    }

    /** Driver override: while true, the gates are held open regardless of the shooter mode or flywheel speed. */
    public void setManualGateOpen(boolean open) {
        manualGateOpen = open;
    }

    /** True once both flywheels have reached speed in SHOOT mode and the gates are open for feeding. */
    public boolean isShootReady() {
        return shootReady;
    }

    public void setState(CaseModes s) {
        if (s != CaseModes.SHOOT) shootReady = false;
        currentMode = s; //currentMode = s??? what does that mean??
    }
//    public double distanceToSpeed(double distanceCm) {
//        speed = lut.getSpeed(distanceCm);
//        distance = distanceCm;
//        return speed;
//    }
    /** Stores the target speeds without spinning the motors; update() applies them in the SHOOT modes. */
    public double getTargetPollenSpeed() {
        return pollenSpeed;
    }

    public void setTargetSpeed(double pollenSpeed){
        this.pollenSpeed = pollenSpeed;
    }
    public void setShooterSpeed(double pollenSpeed){
        this.pollenSpeed = pollenSpeed;
        pollenShooter.setVelocity(pollenSpeed);
    }
//    public String telemetryUpdate() {
//        return "Servo Position: " + hood.getPosition() + " \n ShooterMode: " + currentMode
//                + " \n Shooter Speed: " + getShooterVelocity() + " \n " + "Target Speed/Vel: " + distance + ":" + speed;
//    }
    public double getPollenShooterVelocity()
    {
        return pollenShooter.getVelocity();
    }

    double degreesToPosition(double degrees) {
        return 0.15 + (degrees - 15) / (75 - 15) * (0.70 - 0.15);
    }
    public void closeGate() {
        pollenGate.setPosition(POLLEN_GATE_CLOSED);
    }
    public void openGate()  {
        pollenGate.setPosition(POLLEN_GATE_OPEN);
    }

}
