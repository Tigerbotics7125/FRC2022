/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.tigerlib;

public class Util {

    /**
     * combines {@link #ramp}, {@link #deadband}, and {@link #clamp} into one such that [min,
     * -deadband][deadband, max] and joystick is exponential, preserving sign.
     *
     * @param joystick
     * @param exp
     * @param deadband
     * @param min
     * @param max
     * @return
     */
    public static double scaledDeadbandClamp(
            double joystick, int exp, double deadband, double min, double max) {
        joystick = ramp(joystick, exp);
        joystick = deadband(joystick, deadband);
        joystick = clamp(joystick, min, max);
        return joystick;
    }

    /**
     * Multiplies a value by an exponent, preserving sign.
     *
     * @param joystick
     * @param exp
     * @return
     */
    public static double ramp(double joystick, int exp) {
        if (joystick < 0) {
            return -Math.pow(-joystick, exp);
        } else {
            return Math.pow(joystick, exp);
        }
    }

    /**
     * Applies a deadband such that [-Double.MIN_VALUE, -deadband][deadband, Double.MAX_VALUE]
     *
     * @param joystick
     * @param deadband
     * @return
     */
    public static double deadband(double joystick, double deadband) {
        if (Math.abs(joystick) < deadband) {
            return 0;
        } else {
            return joystick;
        }
    }

    /**
     * Clamps a value to a range such that [min, max]
     *
     * @param joystick
     * @param min
     * @param max
     */
    public static double clamp(double joystick, double min, double max) {
        if (joystick < min) {
            return min;
        } else if (joystick > max) {
            return max;
        } else {
            return joystick;
        }
    }
}
