package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PANELS IS OFF: these do nothing. The real version is in panels_backup/PanelsSupport.java.txt
 * (see panels_backup/README.txt to turn Panels back on).
 */
public final class PanelsSupport {
    private PanelsSupport() {}

    public static Telemetry joined(Telemetry driverStation) {
        return driverStation;
    }

    public static void drawRobot(double x, double y, double headingRad) {
    }
}
