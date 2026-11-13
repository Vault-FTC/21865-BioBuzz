package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * PANELS IS OFF: this stub just hands back the Driver Station gamepads (pad1 = gamepad1, pad2 = gamepad2).
 * The real laptop-gamepad version is in panels_backup/LaptopInput.java.txt (see panels_backup/README.txt).
 */
public class LaptopInput {
    public Gamepad pad1;
    public Gamepad pad2;

    public void update(Gamepad driverStation1, Gamepad driverStation2) {
        pad1 = driverStation1;
        pad2 = driverStation2;
    }

    public String status() {
        return "Panels off";
    }
}
