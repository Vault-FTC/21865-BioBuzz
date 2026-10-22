package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandSystem.Command;

public class WaitCommand extends Command {
    private final ElapsedTime timer = new ElapsedTime();
    private double startTimestamp;

    private final double waitTimeMilliseconds;

    public WaitCommand(double waitTimeMilliseconds) {
        this.waitTimeMilliseconds = waitTimeMilliseconds;
    }

    @Override
    public void initialize() {
        startTimestamp = timer.milliseconds();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > startTimestamp + waitTimeMilliseconds;
    }
}