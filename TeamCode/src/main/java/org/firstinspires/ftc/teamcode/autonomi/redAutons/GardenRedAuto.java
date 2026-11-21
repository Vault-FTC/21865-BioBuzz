package org.firstinspires.ftc.teamcode.autonomi.redAutons;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// Preload shot, garden, second shot, park.
// Everything shared (setup, loops, telemetry, move/shoot helpers, the universal timeout) is in RedAutoTemplate.
@Autonomous(name = "Garden Red Auto", group = "Red")
public class GardenRedAuto extends RedAutoTemplate {

    // Poses (protected so an auto that extends this one can use them, e.g. to override one path)
    protected final Pose start = poseFactory.of(59, 8.5, 90);
    protected final Pose shoot1 = poseFactory.of(59, 19.5, 90);
    protected final Pose garden = poseFactory.of(7.5, 5.5, 0);
    protected final Pose gardenControl1 = poseFactory.of(31.5, 5.5, 180);
    protected final Pose pt1Shoot2 = poseFactory.of(28.5, 100.5, 270);
    protected final Pose pt1Shoot2Control1 = poseFactory.of(26.5, 27.4957, 180);
    protected final Pose pt2Shoot2 = poseFactory.of(58, 127.5, 270);
    protected final Pose pt2Shoot2Control1 = poseFactory.of(61.5, 111.5, 180);
    protected final Pose park = poseFactory.of(14, 96, 0);
    protected final Pose parkControl1 = poseFactory.of(59.5, 111.5, 180);

    @Override
    protected Pose startPose() {
        return start;
    }

    // The whole auto, in the order it runs.
    @Override
    protected Command autoRoutine() {
        return sequential(
                new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED)),   // flywheels on for the whole auto
                move(shoot1(), 250),
                shootCommand(0, 2500),   // preload shot (this auto feeds for 2.5 s)
                move(garden(), 1500),
                move(pt1Shoot2()),
                move(pt2Shoot2()),
                shootCommand(0, 2500),
                move(park())
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

    public Path park() {
        return named("park", curve(pt2Shoot2, parkControl1, park).linear(pt2Shoot2, park));
    }
}
