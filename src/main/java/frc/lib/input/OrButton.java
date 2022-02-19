/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.lib.input;

import edu.wpi.first.wpilibj2.command.button.Button;

/** @author Spectrum 3847 */
public class OrButton extends Button {

    Button b1;
    Button b2;

    public OrButton(Button button, Button button2) {
        b1 = button;
        b2 = button2;
    }

    public boolean get() {
        return b1.get() || b2.get();
    }
}
