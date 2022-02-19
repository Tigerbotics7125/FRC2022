/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmConstants {
    public static final int kArmId = 5;

    public static final MotorType kArmMotorType = MotorType.kBrushed;

    public static final double kSVolts = 0.0; // static gain // TODO:
    public static final double kGVolts = 0.0; // gravity gain // TODO:
    public static final double kVVoltSecondPerRad = 0.0; // velocity gain // TODO:
    public static final double kAVoltSecondSquaredPerRad = 0.0; // acceleration gain // TODO:

    public static final double kP = 0.0; // proportional // TODO:
    public static final double kD = 0.0; // derivative // TODO:
    public static final double kI = 0.0; // integral // TODO:
    public static final double kFF = 0; // feedforward // TODO:

    public static final double kMaxAccelerationRadPerSecSquared = 0.0; // TODO:
    public static final double kArmOffsetRads = 0.0; // TODO:
    public static final double kMaxVelocityRadPerSecond = 0.0; // TODO:
}
