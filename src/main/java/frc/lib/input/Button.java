/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.lib.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.input.XboxGamepad.XboxButton;

/** @author Spectrum 3847 */
public class Button extends JoystickButton {

    public Button(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
    }

    public Button(edu.wpi.first.wpilibj.XboxController joystick, XboxButton button) {
        super(joystick, button.value);
    }

    public Button(XboxGamepad joystick, XboxButton button) {
        super(joystick, button.value);
    }
}
