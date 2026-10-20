package org.firstinspires.ftc.teamcode;

/**
 * A target pose in the shared field frame (see FieldConstants): inches, field center (70, 70),
 * heading in degrees, counter-clockwise positive.
 */
public class Location {
    public double x;
    public double y;
    public double headingDegrees;

    public Location(double x, double y, double headingDegrees) {
        this.x = x;
        this.y = y;
        this.headingDegrees = headingDegrees;
    }

    public Location(double x, double y) {
        this(x, y, 0);
    }
}
