/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.tigerlib;

public class Util {

    /**
     * Multiplies a value by an exponent, preserving sign.
     *
     * @param x input
     * @param exp exponent
     * @return
     */
    public static double ramp(double x, double exp) {
        if (x < 0) {
            return -Math.pow(-x, exp);
        } else {
            return Math.pow(x, exp);
        }
    }

    /**
     * Applies a deadband such that [-Double.MIN_VALUE, -deadband][deadband, Double.MAX_VALUE]
     *
     * @param x input
     * @param d deadband
     * @return
     */
    public static double deadband(double x, double d) {
        if (Math.abs(x) < d) {
            return 0;
        } else {
            return x;
        }
    }

    /**
     * Clamps a value to a range such that [min, max]
     *
     * @param x input
     * @param min
     * @param max
     */
    public static double clamp(double x, double min, double max) {
        if (x < min) {
            return min;
        } else if (x > max) {
            return max;
        } else {
            return x;
        }
    }

    /**
     * combines {@link #ramp}, {@link #deadband}, and {@link #clamp} into one such that [min,
     * -deadband][deadband, max] and joystick is exponential, preserving sign.
     *
     * @param x input
     * @param d deadband
     * @param exp exponent
     * @param min
     * @param max
     * @return
     */
    public static double scaledDeadbandClamp(
            double x, double d, double exp, double min, double max) {
        x = deadband(x, d);
        x = ramp(x, exp);
        x = clamp(x, min, max);
        return x;
    }

    /**
     * Applies {@link deadband} to the input, then applies sensitivity as a power onto the input.
     * Graph available here: https://www.desmos.com/calculator/o91q83dudn
     *
     * @param x input
     * @param d deadband
     * @param s sensitivity
     * @return where x < -deadband, -(x^s), where -deadband <= x <= deadband, 0, and where deadband
     *     < x, x^s
     */
    public static double deadbandSensitivy(double x, double d, double s) {

        x = deadband(x, d);

        if (x < 0) {
            return -Math.pow(x, s);
        } else {
            return Math.pow(x, s);
        }
    }

    /** Applies {@link deadbandSensitivity} to the input, then clamps to nominal joystick range. */
    public static double joystickDeadbandSensitivity(double joystick, double d, double s) {
        joystick = deadbandSensitivy(joystick, d, s);
        joystick = clamp(joystick, -1, 1);
        return joystick;
    }
}
