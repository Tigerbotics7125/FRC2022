/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.old;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

public class DifferentialDrivetrainConstants {
    public static final int kLeftDeviceId = 10;
    public static final int kRightDeviceId = 20;

    public static final double kWheelBaseWidth = Units.inchesToMeters(23);
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final double kGearRatio = 10.71;
    public static final double kRPMToMPSConversionFactor =
            1.0 / (kGearRatio * Math.PI * kWheelDiameter) / 60.0;
    public static final double kDistancePerPulse =
            1.0 / (42 / kGearRatio * (Math.PI * kWheelDiameter));

    public static final double kFeedforwardStaticGain = 0.0; // TODO: tutorial 0.268
    public static final double kFeedforwardVelocityGain = 0.0; // TODO:
}
