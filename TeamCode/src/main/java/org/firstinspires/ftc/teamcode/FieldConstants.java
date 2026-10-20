package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * The single field coordinate system shared by autonomous and teleop.
 *
 *   (0,140) +-----------------+ (140,140)
 *           |                 |
 *        +Y |     (70,70)     |
 *        ^  |                 |
 *        |  +-----------------+
 *     (0,0) --> +X       (140,0)
 *
 * Units: INCHES and DEGREES.
 * Origin (0, 0) is the bottom-left corner of the field. +X is horizontal (to the right),
 * +Y is vertical (up). The field center is (70, 70) and the top-right corner is (140, 140).
 * Heading is measured counter-clockwise from +X: 0 = facing right (+X), 90 = facing up (+Y),
 * 180 = facing left, -90 = facing down. So a robot at heading 90 that drives forward raises Y.
 */
public final class FieldConstants {
    private FieldConstants() {}

    public static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;

    public static final double FIELD_SIZE = 140;
    public static final double FIELD_CENTER_X = FIELD_SIZE / 2;   // 70
    public static final double FIELD_CENTER_Y = FIELD_SIZE / 2;   // 70
    public static final Pose2D FIELD_CENTER =
            new Pose2D(DistanceUnit.INCH, FIELD_CENTER_X, FIELD_CENTER_Y, AngleUnit.DEGREES, 90);

    // Autonomous start poses. Red and blue start in opposite corners, each facing its goal.
    public static final Pose2D RED_START_POSE =
            new Pose2D(DistanceUnit.INCH, 60, 8, AngleUnit.DEGREES, 90);
    public static final Pose2D BLUE_START_POSE =
            new Pose2D(DistanceUnit.INCH, 86, 132, AngleUnit.DEGREES, 270);

    /** Red's autonomous start pose. Alias kept for the red autos, which reference START_POSE. */
    public static final Pose2D START_POSE = RED_START_POSE;

    // Where a teleop-only robot is placed, and where gamepad2 Share resets it to. Same spot and heading
    // as each autonomous start, so a fresh teleop matches the pose autonomous hands over.
    public static final Pose2D RED_TELEOP_START_POSE = RED_START_POSE;
    public static final Pose2D BLUE_TELEOP_START_POSE = BLUE_START_POSE;

    // Field direction (degrees) that pushing the teleop drive stick UP moves the robot toward: away from
    // each alliance's driver station. Red drives toward +Y, blue toward -Y.
    public static final double RED_STICK_UP_HEADING_DEG = 90;
    public static final double BLUE_STICK_UP_HEADING_DEG = 270;

    // Teleop auto-aim points, in field inches. "Case one" is the goal's position at the start of the
    // match; "case two" is after the goal tips (toggled with gamepad2 Square -> cellPosition).
    public static final Pose2D RED_GOAL_CASE_ONE =
            new Pose2D(DistanceUnit.INCH, 60, 60, AngleUnit.DEGREES, 0);
    public static final Pose2D RED_GOAL_CASE_TWO =
            new Pose2D(DistanceUnit.INCH, 60, 85, AngleUnit.DEGREES, 0);
    public static final Pose2D BLUE_GOAL_CASE_ONE =
            new Pose2D(DistanceUnit.INCH, 84, 85, AngleUnit.DEGREES, 0);
    public static final Pose2D BLUE_GOAL_CASE_TWO =
            new Pose2D(DistanceUnit.INCH, 84, 60, AngleUnit.DEGREES, 0);

    /** Wraps an angle in degrees into [-180, 180). */
    public static double normalizeDegrees(double degrees) {
        double d = (degrees + 180) % 360;
        if (d < 0) d += 360;
        return d - 180;
    }
}
