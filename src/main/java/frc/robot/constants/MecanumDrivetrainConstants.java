/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * Contains all mecanum drivetrain constants.
 * 
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class MecanumDrivetrainConstants {

    // Body Constants
    // [0, 1] will limit max speed
    public static final double kMaxSpeed = .75;
    public static final double kWheelDiameter = Units.inchesToMeters(6);

    // Joystick Constants
    public static final double kDeadband = 0.2;
    public static final double kSensitivity = 2.8;

    // CAN IDs
    public static final int kFrontLeftId = 1;
    public static final int kRearLeftId = 2;
    public static final int kFrontRightId = 3;
    public static final int kRearRightId = 4;

    // Motor Type
    public static final CANSparkMaxLowLevel.MotorType kMotorType = CANSparkMaxLowLevel.MotorType.kBrushless;

    // Offset in meters from center of robot (also imu)
    public static final Translation2d kFrontLeftOffset = new Translation2d(Units.inchesToMeters(10.18),
            Units.inchesToMeters(-10.857));
    public static final Translation2d kRearLeftOffset = new Translation2d(Units.inchesToMeters(-10.18),
            Units.inchesToMeters(-10.857));
    public static final Translation2d kFrontRightOffset = new Translation2d(Units.inchesToMeters(10.18),
            Units.inchesToMeters(10.857));
    public static final Translation2d kRearRightOffset = new Translation2d(Units.inchesToMeters(-10.18),
            Units.inchesToMeters(10.857));

    // autonomous constants
    public static final double kMaxAutoVelocity = 8.0; // m/s
    public static final double kMaxAutoAcceleration = 5.0; // m/s^2
    public static final PIDController kXPID = new PIDController(1, 0, 0);
    public static final PIDController kYPID = new PIDController(1, 0, 0);
    public static final ProfiledPIDController kThetaPID = new ProfiledPIDController(1, 0, 0,
            new Constraints(6.28, 3.14));

    // Gearbox Constants
    public static final double kGearRatio = 10.71;
    public static final double kRPMtoMPSConversionFactor = 1.0 / (kGearRatio * (Math.PI * kWheelDiameter));
    public static final double kDistancePerPulse = 1.0
            / (42.0 /* Hall Effects CPR */ / kGearRatio * (Math.PI * kWheelDiameter));

    /**
     * Theoretical max wheel speeds
     * 
     * Calculated from:
     * https://x-engineer.org/calculate-wheel-vehicle-speed-engine-speed/
     * https://en.ans.wiki/105/how-to-convert-angular-velocity-in-revolutions-per-minute-rpm-to-linear-speed-in-meters-per-second-m-slash-s/
     * 
     * please kill me
     */
    public static final int kNEOFreeSpeedRPM = 5676;
    public static final double kMaxWheelSpeedRPM = 529.97;
    public static final double kMaxWheelSpeedMPS = ((2 * Math.PI * kWheelDiameter) / 60) * kMaxWheelSpeedRPM;
}
