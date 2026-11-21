package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Complete Blue Auto", group = "Blue")
public class CompleteBlueAuto extends BlueAutoTemplate {

    protected final Pose start = poseFactory.of(82.5, 133, 270);
    protected final Pose shoot1 = poseFactory.of(82.5, 114.5, 270);
    protected final Pose garden = poseFactory.of(131.5, 136, 180);
    protected final Pose gardenControl1 = poseFactory.of(110, 136, 0);
    protected final Pose pt1Shoot2 = poseFactory.of(113, 41, 90);
    protected final Pose pt1Shoot2Control1 = poseFactory.of(115, 114.0043, 0);
    protected final Pose pt2Shoot2 = poseFactory.of(82.5, 18, 90);
    protected final Pose pt2Shoot2Control1 = poseFactory.of(80, 30, 0);
    protected final Pose flower1 = poseFactory.of(92, 13.5, 90);
    protected final Pose flower1Control1 = poseFactory.of(85., 20., 180);
    protected final Pose shoot3 = poseFactory.of(82.5, 16, 90);
    protected final Pose shoot3Control1 = poseFactory.of(81, 23, 180);
    protected final Pose flower2 = poseFactory.of(126.25, 95.5, 180);
    protected final Pose flower2Control1 = poseFactory.of(70, 9, 180);
    protected final Pose flower2Control2 = poseFactory.of(140, 48, 180);
    protected final Pose flower2Control3 = poseFactory.of(102, 95, 180);
    protected final Pose shoot4 = poseFactory.of(82.5, 114.5, 270);
    protected final Pose park = poseFactory.of(127.5, 45.5, 180);
    protected final Pose parkControl1 = poseFactory.of(117, 102, 180);

    @Override protected Pose startPose() {
        return start;
    }

    // The whole auto, in the order it runs. Flywheels spin up first, then a preload shot, then a shot after each shoot path.
    @Override protected Command autoRoutine() {
        return sequential(
                new InstantCommand(() -> shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED)),   // flywheels on for the whole auto
                move(shoot1(), 350),
                shootCommand(),   // preload shot
                move(garden(), 2000),
                move(pt1Shoot2(), 1750),
                move(pt2Shoot2(), 1250),
                shootCommand(),
                armDown(),
                move(flower1(), 2000),
                move(shoot3(), 500),
                shootCommand(),
                move(flower2(), 5750),   // arm stays down from flower 1
                move(shoot4(), 2000),
                shootCommand(),
                move(park(), ArmMove.UP)   // arm comes up while driving
        );
    }

    // Each path starts from the END pose of the path before it in autoRoutine(), so the heading and position carry
    // over. If you reorder or remove steps (for example drop pt2Shoot2 and go straight from pt1Shoot2 to park),
    // change that path's start pose to the end pose of whatever path now comes before it.
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
