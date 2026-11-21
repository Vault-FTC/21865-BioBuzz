package org.firstinspires.ftc.teamcode.autonomi.redAutons;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// Flywheels spin up first, then a preload shot, then a shot after each shoot path.
// Everything shared (setup, loops, telemetry, move/shoot helpers, the universal timeout) is in RedAutoTemplate.
@Autonomous(name = "Complete Red Auto", group = "Red")
public class CompleteRedAuto extends RedAutoTemplate {

    // Poses (protected so an auto that extends this one can use them, e.g. to override one path)
    protected final Pose start = poseFactory.of(59, 8.5, 90);
    protected final Pose shoot1 = poseFactory.of(59, 20, 90);
    protected final Pose garden = poseFactory.of(7.5, 5.5, 0);
    protected final Pose gardenControl1 = poseFactory.of(31.5, 5.5, 180);
    protected final Pose pt1Shoot2 = poseFactory.of(28.5, 100.5, 270);
    protected final Pose pt1Shoot2Control1 = poseFactory.of(26.5, 27.4957, 180);
    protected final Pose pt2Shoot2 = poseFactory.of(61, 120, 270);
    protected final Pose pt2Shoot2Control1 = poseFactory.of(61.5, 111.5, 180);
    protected final Pose flower1 = poseFactory.of(49, 127.5, 270);
    protected final Pose flower1Control1 = poseFactory.of(56.5, 121.5, 0);
    protected final Pose shoot3 = poseFactory.of(59, 120, 270);
    protected final Pose shoot3Control1 = poseFactory.of(60.0744, 118.824, 0);
    protected final Pose flower2 = poseFactory.of(14, 46, 0);
    protected final Pose flower2Control1 = poseFactory.of(71.9061, 132.971, 0);
    protected final Pose flower2Control2 = poseFactory.of(1.0251, 93.1901, 0);
    protected final Pose flower2Control3 = poseFactory.of(39.1327, 43.127, 0);
    protected final Pose shoot4 = poseFactory.of(59, 20, 90);
    protected final Pose park = poseFactory.of(14, 96, 90);
    protected final Pose parkControl1 = poseFactory.of(24.5, 39.5, 0);

    @Override
    protected Pose startPose() {
        return start;
    }

    // The whole auto, in the order it runs.
    @Override
    protected Command autoRoutine() {
        return sequential(
                new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED)),   // flywheels on for the whole auto
                move(shoot1(), 350),
                shootCommand(),   // preload shot
                move(garden(), 1850),
                move(pt1Shoot2(), 1750),
                move(pt2Shoot2(), 1250),
                shootCommand(),
                armDown(),
                move(flower1(), 2000),
                move(shoot3(), 500),
                shootCommand(),
                move(flower2(), 5500),   // arm stays down from flower 1
                move(shoot4(), 2000),
                shootCommand(),
                move(park(), ArmMove.UP)   // arm comes up while driving
        );
    }

    // Each path starts from the END pose of the path before it in autoRoutine(). The first path starts from
    // startPose(), so an auto that extends this one and changes the start also changes the first path.
    public Path shoot1() {
        return named("shoot1", line(startPose(), shoot1).linear(startPose(), shoot1));
    }

    public Path garden() {
        return named("garden", curve(shoot1, gardenControl1, garden).linear(shoot1, garden, 0.5));   // turn finishes halfway, then holds
    }

    public Path pt1Shoot2() {
        return named("pt1Shoot2", curve(garden, pt1Shoot2Control1, pt1Shoot2).linear(garden, pt1Shoot2));
    }

    public Path pt2Shoot2() {
        return named("pt2Shoot2", curve(pt1Shoot2, pt2Shoot2Control1, pt2Shoot2).linear(pt1Shoot2, pt2Shoot2));
    }

    public Path flower1() {
        return named("flower1", curve(pt2Shoot2, flower1Control1, flower1).linear(pt2Shoot2, flower1));
    }

    public Path shoot3() {
        return named("shoot3", curve(flower1, shoot3Control1, shoot3).linear(flower1, shoot3));
    }

    public Path flower2() {
        return named("flower2", curve(shoot3, flower2Control1, flower2Control2, flower2Control3, flower2).linear(shoot3, flower2));
    }

    public Path shoot4() {
        return named("shoot4", line(flower2, shoot4).linear(flower2, shoot4));
    }

    public Path park() {
        return named("park", curve(shoot4, parkControl1, park).linear(shoot4, park));
    }
}
