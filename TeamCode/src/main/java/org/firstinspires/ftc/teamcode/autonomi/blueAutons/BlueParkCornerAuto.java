package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Park auto that starts from the corner. Everything else (park pose, path, timeout) comes from BlueParkTemplate.
@Autonomous(name = "Blue Park Corner", group = "Blue")
public class BlueParkCornerAuto extends BlueParkTemplate {

    @Override
    protected Pose startPose() {
        return poseFactory.of(136, 9, 0);
    }
}
