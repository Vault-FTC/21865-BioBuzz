package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class WheelDirectionOpMode extends LinearOpMode {
    // Driver Station and Panels (laptop) gamepads combined: read pad1 / pad2 instead of gamepad1 / gamepad2
    private final org.firstinspires.ftc.teamcode.subsystems.LaptopInput input = new org.firstinspires.ftc.teamcode.subsystems.LaptopInput();
    private com.qualcomm.robotcore.hardware.Gamepad pad1;
    private com.qualcomm.robotcore.hardware.Gamepad pad2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = org.firstinspires.ftc.teamcode.subsystems.PanelsSupport.joined(telemetry);   // also show telemetry on Panels
        input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0.0, 31.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
            pinpoint.update();
            doMotor(pad1.a, frontLeftMotor, "frontLeftMotor");
            doMotor(pad1.x, frontRightMotor, "frontRightMotor");
            doMotor(pad1.b, backLeftMotor, "backLeftMotor");
            doMotor(pad1.y, backRightMotor, "backRightMotor");

            telemetry.addData("Pos X (mm)", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("Pos Y (mm)", pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    public void doMotor(boolean button, DcMotorEx motor, String name) {
        if (button) {
            motor.setPower(0.4);
            telemetry.addData(name, " : active");
        } else {
            motor.setPower(0);
            telemetry.addData(name, " : off");
        }
    }
}