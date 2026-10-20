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

    // Where the robot starts and which way it faces: (59, 8.5) facing up the field (+Y), the same
    // start redAuto uses. The heading matters: field-relative moves (like "decrease X") only go the
    // right physical direction if this heading matches how the robot is actually placed.
    public static final Pose2D START_POSE =
            new Pose2D(DistanceUnit.INCH, 59, 8.5, AngleUnit.DEGREES, 90);

    // Pose a standalone teleop starts from. The robot is placed turned 180 degrees from the autonomous start,
    // so this uses the true (reversed) heading. That keeps the heading consistent with the pose autonomous
    // hands over through PoseStorage, which is also a true heading.
    public static final Pose2D TELEOP_START_POSE =
            new Pose2D(DistanceUnit.INCH, START_POSE.getX(DistanceUnit.INCH), START_POSE.getY(DistanceUnit.INCH),
                    AngleUnit.DEGREES, normalizeDegrees(START_POSE.getHeading(AngleUnit.DEGREES) + 180));

    // Field direction (degrees) that pushing the teleop drive stick UP moves the robot toward. With a true
    // heading this is the same whether teleop starts on its own or after autonomous.
    public static final double TELEOP_STICK_UP_HEADING_DEG = normalizeDegrees(START_POSE.getHeading(AngleUnit.DEGREES));

    // Blue goal aim points: 105 cm in front of and 83 cm (or 60 cm) to the left of the robot's
    // starting position, as originally measured. Converted here to field inches.
    public static final Pose2D BLUE_GOAL_CASE_ONE = offsetFromStart(DistanceUnit.CM, 105, 83);
    public static final Pose2D BLUE_GOAL_CASE_TWO = offsetFromStart(DistanceUnit.CM, 105, 60);

    /** A field pose located (forward, left) away from START_POSE, relative to the way the robot starts facing. */
    public static Pose2D offsetFromStart(DistanceUnit unit, double forward, double left) {
        double h = START_POSE.getHeading(AngleUnit.RADIANS);
        double dx = DISTANCE_UNIT.fromUnit(unit, forward);
        double dy = DISTANCE_UNIT.fromUnit(unit, left);
        double x = START_POSE.getX(DISTANCE_UNIT) + dx * Math.cos(h) - dy * Math.sin(h);
        double y = START_POSE.getY(DISTANCE_UNIT) + dx * Math.sin(h) + dy * Math.cos(h);
        return new Pose2D(DISTANCE_UNIT, x, y, AngleUnit.RADIANS, 0);
    }

    /** Wraps an angle in degrees into [-180, 180). */
    public static double normalizeDegrees(double degrees) {
        double d = (degrees + 180) % 360;
        if (d < 0) d += 360;
        return d - 180;
    }
}
