/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsys {
    AddressableLED mLeds = new AddressableLED(0);

    public LEDSubsys() {
        mLeds.setLength(60);
    }

    public void periodic() {
        AddressableLEDBuffer b = new AddressableLEDBuffer(60);
        b.setRGB(0, 255, 0, 0);
        mLeds.setData(b);
    }
}
