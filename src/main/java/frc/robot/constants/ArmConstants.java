/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {

    public static final int kId = 0;

    public static final double kSVolts = 0.0; // static gain // TODO:
    public static final double kGVolts = 0.0; // gravity gain // TODO:
    public static final double kVVoltSecondPerRad = 0.0; // velocity gain // TODO:
    public static final double kAVoltSecondSquaredPerRad = 0.0; // acceleration gain // TODO:

    public static final double kPlanetaryGearboxRatio = 81.0; // 81:1
    public static final double kGearAndChainRatio = 1; // TODO: idk yet.

    public static final Constraints kTrapezoidProfile =
            new Constraints(.25, .1); // radians per second, and radians per second^2

    public static final double kArmOffsetRads = 0.0; // TODO:
}
