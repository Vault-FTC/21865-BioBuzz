package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.prism.LayerHeight;
import org.firstinspires.ftc.teamcode.prism.PrismAnimations;

/**
 * goBILDA Prism LED strip, one solid color across the whole strip.
 *
 * <p>Robot config: add the Prism on an I2C bus as "goBILDA® Prism RGB LED Driver" named {@link #DEVICE_NAME}.
 * If it isn't in the config the LEDs are just skipped (no crash), and {@link #isConnected()} is false.
 */
public class LEDs extends Subsystem {
    public static final String DEVICE_NAME = "prism";

    private final GoBildaPrismDriver prism;
    private final PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.TRANSPARENT);
    private Color currentColor = null;

    public LEDs(HardwareMap hardwareMap) {
        prism = hardwareMap.tryGet(GoBildaPrismDriver.class, DEVICE_NAME);
        if (prism != null) {
            prism.clearAllAnimations();   // drop the boot animation so only our solid color shows
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);
            currentColor = Color.TRANSPARENT;
        }
    }

    public boolean isConnected() {
        return prism != null;
    }

    /** Sets the whole strip to {@code color}. Only talks to the Prism when the color actually changes (I2C is slow). */
    public void setColor(Color color) {
        if (prism == null || color == currentColor) return;
        solid.setPrimaryColor(color);
        prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
        currentColor = color;
    }
}
