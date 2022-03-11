/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.tigerlib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * An input class for the logitech flight joystick that actually makes sense.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class GExtreme3DProJoystick extends GenericHID {

    /** Represents an analog axis on the joystick */
    public enum AxisType {
        kX(0),
        kY(1),
        kZ(2),
        kThrottle(3),
        kDpadX(5),
        kDpadY(6);

        public final int value;

        AxisType(int value) {
            this.value = value;
        }
    }

    /** Represents a digital button on the joystick */
    public enum ButtonType {
        kTrigger(1),
        kThumb(2),
        kTop3(3),
        kTop4(4),
        kTop5(5),
        kTop6(6),
        kBottom7(7),
        kBottom8(8),
        kBottom9(9),
        kBottom10(10),
        kBottom11(11),
        kBottom12(12);

        public final int value;

        ButtonType(int value) {
            this.value = value;
        }
    }

    public final Button kTrigger = new Button(() -> getRawButton(ButtonType.kTrigger.value));
    public final Button kThumb = new Button(() -> getRawButton(ButtonType.kThumb.value));
    public final Button kTop3 = new Button(() -> getRawButton(ButtonType.kTop3.value));
    public final Button kTop4 = new Button(() -> getRawButton(ButtonType.kTop4.value));
    public final Button kTop5 = new Button(() -> getRawButton(ButtonType.kTop5.value));
    public final Button kTop6 = new Button(() -> getRawButton(ButtonType.kTop6.value));
    public final Button kBottom7 = new Button(() -> getRawButton(ButtonType.kBottom7.value));
    public final Button kBottom8 = new Button(() -> getRawButton(ButtonType.kBottom8.value));
    public final Button kBottom9 = new Button(() -> getRawButton(ButtonType.kBottom9.value));
    public final Button kBottom10 = new Button(() -> getRawButton(ButtonType.kBottom10.value));
    public final Button kBottom11 = new Button(() -> getRawButton(ButtonType.kBottom11.value));
    public final Button kBottom12 = new Button(() -> getRawButton(ButtonType.kBottom12.value));

    /**
     * Constructs a new instance of the joystick
     *
     * @param port The port index on Driver Station that the joystick is plugged into.
     */
    public GExtreme3DProJoystick(int port) {
        super(port);
    }

    /**
     * Get the X value of the joystick.
     *
     * <p>Right is positive.
     *
     * @return The X value of the joystick.
     */
    public double getX() {
        return -super.getRawAxis(AxisType.kX.value);
    }

    /**
     * Get the Y value of the joystick.
     *
     * <p>Forward is positive.
     *
     * @return The Y value of the joystick.
     */
    public double getY() {
        return -super.getRawAxis(AxisType.kY.value);
    }

    /**
     * Get the Z value of the joystick.
     *
     * <p>Clcok-wise is positive.
     *
     * @return The Z value of the joystick.
     */
    public double getZ() {
        return super.getRawAxis(AxisType.kZ.value);
    }

    /**
     * Get the throttle of the joystick.
     *
     * <p>Up Is positive.
     *
     * @return The throttle position of the joystick.
     */
    public double getThrottle() {
        return super.getRawAxis(AxisType.kThrottle.value);
    }

    /**
     * Gets the POV of the dpad; in 45 degree increments CW from the top.
     *
     * <p>For instance: up is 0, up & right is 45, right is 90, etc.
     *
     * @return The POV of the dpad.
     */
    public int getPOV() {
        int x = (int) -super.getRawAxis(AxisType.kDpadX.value);
        int y = (int) super.getRawAxis(AxisType.kDpadY.value);

        if (x == 1 && y == 0) {
            return 0;
        } else if (x == 1 && y == 1) {
            return 45;
        } else if (x == 0 && y == 1) {
            return 90;
        } else if (x == -1 && y == 1) {
            return 135;
        } else if (x == -1 && y == 0) {
            return 180;
        } else if (x == -1 && y == -1) {
            return 225;
        } else if (x == 0 && y == -1) {
            return 270;
        } else if (x == 1 && y == -1) {
            return 315;
        } else {
            return -1;
        }
    }

    /**
     * Get the magnitude of the direction vector formed by the joystick's current position relative
     * to its origin.
     *
     * @return The magnitude of the direction vector
     */
    public double getMagnitude() {
        return Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2));
    }

    /**
     * Get the direction of the vector formed by the joystick and its origin in radians.
     *
     * @return The direction of the vector in radians
     */
    public double getDirectionRadians() {
        return Math.atan2(getX(), -getY());
    }

    /**
     * Get the direction of the vector formed by the joystick and its origin in degrees.
     *
     * @return The direction of the vector in degrees
     */
    public double getDirectionDegrees() {
        return Math.toDegrees(getDirectionRadians());
    }
}
