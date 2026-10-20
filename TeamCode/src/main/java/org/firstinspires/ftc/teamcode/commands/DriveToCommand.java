// DISABLED (teleop-only): uncomment when working on auto
//package org.firstinspires.ftc.teamcode.commands;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Location;
//import org.firstinspires.ftc.teamcode.commandSystem.Command;
//import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
//
//public class DriveToCommand extends Command {
//    Telemetry telemetry;
//    private final Drivebase drivebase;
//    private final Location location;
//
//    public DriveToCommand(Drivebase drivebase, Location location, Telemetry telemetry) {
//        this.drivebase = drivebase;
//        this.location = location;
//        this.telemetry = telemetry;
//        addRequirements(this.drivebase);
//    }
//
//    @Override
//    public void execute() {
//        drivebase.driveToPosition(location, telemetry);
//        telemetry.addData("Running", "Drive Command");
//    }
//
//    public boolean isFinished() {
//        return drivebase.isAtPosition(location, 5, 5);
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        drivebase.drive(0,0,0);
//    }
//}
