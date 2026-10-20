// DISABLED (teleop-only): uncomment when working on auto
//package org.firstinspires.ftc.teamcode.commands;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Location;
//import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
//
//public class DriveToCommandWaiting extends DriveToCommand{
//    ElapsedTime time = new ElapsedTime();
//    public DriveToCommandWaiting(Drivebase drive, Location target, Telemetry telemetry) {
//        super(drive, target, telemetry);
//    }
//
//    @Override
//    public void initialize() {
//       time.reset();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return time.milliseconds() > 300;
//    }
//
//}
