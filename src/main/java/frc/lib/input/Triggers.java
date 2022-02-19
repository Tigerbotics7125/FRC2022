/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.lib.input;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.input.XboxGamepad.XboxAxis;

/** @author Spectrum 3847 */
public class Triggers {
    Joystick controller;

    public Triggers(Joystick controller) {
        this.controller = controller;
    }

    public double getLeft() {
        if (this.controller.isConnected()) {
            return this.controller.getRawAxis(XboxAxis.LEFT_TRIGGER.value);
        } else {
            return 0;
        }
    }

    public double getRight() {
        if (this.controller.isConnected()) {
            return this.controller.getRawAxis(XboxAxis.RIGHT_TRIGGER.value);
        } else {
            return 0;
        }
    }

    public double getTwist() {
        return -getLeft() + getRight();
    }
}
