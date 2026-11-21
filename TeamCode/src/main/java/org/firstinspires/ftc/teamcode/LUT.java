package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Map;
import java.util.TreeMap;

/**
 * Lookup table (LUT): shot distance -> flywheel speed. Based on the Miami Competition robot's LUT.
 *
 * <p>Distances are in INCHES (field units), from the robot's center to the cell it is aiming at.
 * Speeds are flywheel ticks/s. Between two entries the speed is blended in a straight line, so 30 in
 * with entries at 24 and 36 gets the value halfway between. Closer than the first entry uses the first
 * speed; farther than the last uses the last speed.
 *
 * <p>To tune: drive to a distance (teleop telemetry shows "Distance to cell"), adjust speed with the
 * gamepad2 d-pad until shots go in, and put that distance and speed in below. Entries can be any
 * distances in any order, and you can add or remove as many as you like. The entries are in blueLeft()
 * and blueRight() below.
 */
public class LUT {
    private final TreeMap<Double, Double> speedByDistance = new TreeMap<>();

    /**
     * The two tables, named from the BLUE driver station: "left" and "right" are the blue left and right
     * cells. Red uses the same tables with the sides swapped (red's left cell is blue's right cell, the same
     * physical side of the hive), so tune both from the blue side and red picks them up automatically.
     */
    public static LUT blueLeft() {
        LUT lut = new LUT();
        // Tuned 2026-11-18 (copied into both tables; retune each side on its own)
        lut.addSpeed(28.6, 1076);
        lut.addSpeed(35.3, 1155);
        lut.addSpeed(43.5, 1256);
        lut.addSpeed(60.8, 1394);
        return lut;
    }

    public static LUT blueRight() {
        LUT lut = new LUT();
        // Tuned 2026-11-18 (copied into both tables; retune each side on its own)
        lut.addSpeed(30.4, 1161);
        lut.addSpeed(39.3, 1197);
        lut.addSpeed(47.2, 1234);
        return lut;
    }

    public void addSpeed(double distanceIn, double speed) {
        speedByDistance.put(distanceIn, speed);
    }

    /** Flywheel speed for a shot from distanceIn inches away. */
    public double getSpeed(double distanceIn) {
        Map.Entry<Double, Double> below = speedByDistance.floorEntry(distanceIn);     // closest entry at or below
        Map.Entry<Double, Double> above = speedByDistance.ceilingEntry(distanceIn);   // closest entry at or above
        if (below == null) return above.getValue();   // closer than the first entry
        if (above == null) return below.getValue();   // farther than the last entry
        if (below.getKey().equals(above.getKey())) return below.getValue();   // exactly on an entry

        // Straight-line blend between the two entries
        double fraction = (distanceIn - below.getKey()) / (above.getKey() - below.getKey());
        return below.getValue() + fraction * (above.getValue() - below.getValue());
    }
}
