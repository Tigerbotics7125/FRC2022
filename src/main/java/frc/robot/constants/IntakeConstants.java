/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel;

/**
 * Contains all intake constants.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class IntakeConstants {

    // CAN ID
    public static final int kId = 5;

    // Motor Type
    public static final CANSparkMaxLowLevel.MotorType kMotorType =
            CANSparkMaxLowLevel.MotorType.kBrushed;
}
