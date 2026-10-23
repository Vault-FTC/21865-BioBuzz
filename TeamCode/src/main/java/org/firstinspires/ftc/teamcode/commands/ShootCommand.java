package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * One shot: spin the flywheels up, open the gates and feed once they are at speed, keep feeding for feedMs, then
 * clean up (gates closed, intake off, speeds put back).
 *
 * <p>spinUpTimeoutMs is how long to wait for the flywheels to reach speed. If they get there in time it feeds
 * normally. If the time runs out first, it feeds anyway instead of skipping the shot. 0 or less means wait as long
 * as it takes.
 *
 * <p>The cleanup is in end(), so it runs however the command stops: finished normally, or cancelled by a timeout.
 */
public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final double pollenSpeed;
    private final double nectarSpeed;
    private final double spinUpTimeoutMs;
    private final double feedMs;

    private final ElapsedTime phaseTimer = new ElapsedTime();
    private boolean feeding;
    private double previousPollenSpeed;
    private double previousNectarSpeed;

    public ShootCommand(Shooter shooter, Intake intake, double pollenSpeed, double nectarSpeed,
                        double spinUpTimeoutMs, double feedMs) {
        this.shooter = shooter;
        this.intake = intake;
        this.pollenSpeed = pollenSpeed;
        this.nectarSpeed = nectarSpeed;
        this.spinUpTimeoutMs = spinUpTimeoutMs;
        this.feedMs = feedMs;
    }

    @Override
    public void initialize() {
        previousPollenSpeed = shooter.getTargetPollenSpeed();
        previousNectarSpeed = shooter.getTargetNectarSpeed();
        shooter.setTargetSpeeds(pollenSpeed, nectarSpeed);
        shooter.setState(Shooter.CaseModes.AUTO_SHOOT);
        feeding = false;
        phaseTimer.reset();
    }

    @Override
    public void execute() {
        if (feeding) return;

        boolean ready = shooter.isShootReady();
        boolean timedOut = spinUpTimeoutMs > 0 && phaseTimer.milliseconds() >= spinUpTimeoutMs;
        if (ready || timedOut) {
            if (!ready) {
                // Not at speed but out of time: feed anyway. SHOOT_GATE_CLOSED keeps the flywheels at their target
                // speeds and leaves the intake alone, so the forced gate and intake settings stick.
                shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
                shooter.setManualGateOpen(true);
                intake.setState(Intake.CaseModes.ON);
            }
            feeding = true;
            phaseTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return feeding && phaseTimer.milliseconds() >= feedMs;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setManualGateOpen(false);
        shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
        intake.setState(Intake.CaseModes.OFF);
        shooter.setTargetSpeeds(previousPollenSpeed, previousNectarSpeed);
    }
}
