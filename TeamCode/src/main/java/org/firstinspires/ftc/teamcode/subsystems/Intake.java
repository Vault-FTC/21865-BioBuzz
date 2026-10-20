package org.firstinspires.ftc.teamcode.subsystems;

import android.animation.RectEvaluator;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;

public class Intake extends Subsystem {
    public enum CaseModes
    {
        OFF, REVERSE, ON, HALF_SPEED, SIXTY_PERCENT_SPEED
    }
    private DcMotorEx intake;
    private RevColorSensorV3 frontColorSensor, midColorSensor, backColorSensor;
    CaseModes currentMode = CaseModes.OFF;


    //
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        intake.setDirection(DcMotorEx.Direction.REVERSE);
//        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "frontColorSensor");
//        midColorSensor = hardwareMap.get(RevColorSensorV3.class, "midColorSensor");
//        backColorSensor = hardwareMap.get(RevColorSensorV3.class, "backColorSensor");
    }

    @Override
    public void periodic() {
        update();
    }

    public void update()
    {
        switch(currentMode) {
            case ON:
                intake.setPower(1);
                break;
            case OFF:
                intake.setPower(0);
                break;
            case HALF_SPEED:
                intake.setPower(0.5);
                break;
            case SIXTY_PERCENT_SPEED:
                intake.setPower(0.6);
                break;
            case REVERSE:
                intake.setPower(-1);
                break;
        }
    }

    public void setState(CaseModes s) {
        currentMode = s;
    }

//    public boolean isArtifact(RevColorSensorV3 revColorSensorV3) {
//        boolean greenArtifact;
//        boolean purpleArtifact;
//
//        int r = revColorSensorV3.red();
//        int g = revColorSensorV3.green();
//        int b = revColorSensorV3.blue();
//
//        if ()
//
//
//    }

//    public double numberOfArtifacts() {
//        if (backColorSensor.getNormalizedColors() {
//            return 2;
//        }
//        else if (!frontColorSensor.getState() && breakbeam2.getState()) {
//            return 1;
//        } else {
//            return 0;
//        }
//    }
}
