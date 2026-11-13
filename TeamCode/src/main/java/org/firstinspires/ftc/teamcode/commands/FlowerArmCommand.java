package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Moves the flower arm up or down, then waits waitMs before the next command starts. The servo can't report where it
 * is, so the wait is a fixed time: make it long enough for the arm to finish moving.
 */
public class FlowerArmCommand extends Command {
    private final Intake intake;
    private final boolean up;
    private final double waitMs;
    private final ElapsedTime waitTimer = new ElapsedTime();

    public FlowerArmCommand(Intake intake, boolean up, double waitMs) {
        this.intake = intake;
        this.up = up;
        this.waitMs = waitMs;
    }

    @Override
    public void initialize() {
        intake.setFlowerArmUp(up);
        waitTimer.reset();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return waitTimer.milliseconds() >= waitMs;
    }
}
