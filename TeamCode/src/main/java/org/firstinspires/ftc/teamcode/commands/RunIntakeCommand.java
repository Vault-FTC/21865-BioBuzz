package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/** Runs the intake for runMs, then turns it off. The intake is also turned off if the command is cancelled. */
public class RunIntakeCommand extends Command {
    private final Intake intake;
    private final double runMs;
    private final ElapsedTime runTimer = new ElapsedTime();

    public RunIntakeCommand(Intake intake, double runMs) {
        this.intake = intake;
        this.runMs = runMs;
    }

    @Override
    public void initialize() {
        intake.setState(Intake.CaseModes.ON);
        runTimer.reset();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return runTimer.milliseconds() >= runMs;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(Intake.CaseModes.OFF);
    }
}
