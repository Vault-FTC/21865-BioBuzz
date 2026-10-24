package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;

public class Intake extends Subsystem {
    public enum CaseModes
    {
        OFF, REVERSE, ON, HALF_SPEED, SIXTY_PERCENT_SPEED
    }
    public static final double FLOWER_ARM_UP = 0;
    public static final double FLOWER_ARM_DOWN = 0.51;

    private DcMotorEx intake;
    private Servo flowerArm; // uncomment out flower intake lines when we actually get flower intake into the code
    private RevColorSensorV3 frontColorSensor, midColorSensor, backColorSensor;
    private boolean flowerArmUp = true;
    CaseModes currentMode = CaseModes.OFF;


    //
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flowerArm = hardwareMap.get(Servo.class, "flowerArm");
//        intake.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        update();
    }

    public void update()
    {
        switch(currentMode) {
            case ON:
                intake.setPower(0.9);
                break;
            case OFF:
                intake.setPower(0);
                break;
            case HALF_SPEED:
                intake.setPower(0.5);
                break;
            case SIXTY_PERCENT_SPEED:
                intake.setPower(0.5);
                break;
            case REVERSE:
                intake.setPower(-0.6);
                break;
        }

        flowerArm.setPosition(flowerArmUp ? FLOWER_ARM_UP : FLOWER_ARM_DOWN);
    }

    public void setState(CaseModes s) {
        currentMode = s;
    }

    public void setFlowerArmUp(boolean up) {
        flowerArmUp = up;
    }

    public void toggleFlowerArm() {
        flowerArmUp = !flowerArmUp;
    }

    public boolean isFlowerArmUp() {
        return flowerArmUp;
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
