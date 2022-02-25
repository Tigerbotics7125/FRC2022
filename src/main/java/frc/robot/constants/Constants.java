/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

/**
 * A class to contain all things that remain constant and should be easily changed, without breaking
 * other code.
 *
 * <p>All measurements should be in SI units.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Constants {

    // Misc

    // multiple CAN devices can share, basically as long as they are different types
    // of products / manufactorers, read wpi CAN docs.
    public static final int kPowerDistributionPanelId = 0;
    public static final int kPigeonId = 0;
}
