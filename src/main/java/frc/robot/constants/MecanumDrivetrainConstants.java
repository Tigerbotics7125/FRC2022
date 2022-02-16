package frc.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class MecanumDrivetrainConstants {

  public static final double kMaxSpeed = 3.0; // 3 meters per seconds
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // CAN IDs
  public static final int kFrontLeftId = 1;
  public static final int kRearLeftId = 2;
  public static final int kFrontRightId = 3;
  public static final int kRearRightId = 4;

  // The motor type used for the drive motors
  public static final MotorType kMotorType = MotorType.kBrushless;

  // Offset in meters from center of robot (also imu)
  
  public static final Translation2d kFrontLeftOffset = new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(-10.857));
  public static final Translation2d kRearLeftOffset = new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(-10.857));
  public static final Translation2d kFrontRightOffset = new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(10.857));
  public static final Translation2d kRearRightOffset = new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(10.857));

  // General constants
  public static final double kWheelDiameter = Units.inchesToMeters(6);
  public static final double kGearRatio = 10.71;

  // Conversion factors used for encoder ticks to various units.
  public static final double kRPMtoMPSConversionFactor =
      1.0 / (kGearRatio * (Math.PI * kWheelDiameter));
  public static final double kDistancePerPulse =
      1.0 / (42.0 /* Hall Effects CPR */ / kGearRatio * (Math.PI * kWheelDiameter));
}
