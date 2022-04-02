/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.tigerlib.ledmatrix.Gif;
import java.util.LinkedList;
import java.util.List;

/**
 * Contains all constant variables for subsystems.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Constants {

    // CAN IDs
    public static final int kPigeonId = 0;

    public enum AutoTrajectory {
        TWO_BALL("2BallAuto");

        public final List<PathPlannerTrajectory> kPaths;

        private AutoTrajectory(String... pathNames) {
            kPaths = new LinkedList<PathPlannerTrajectory>();
            for (String name : pathNames) {
                kPaths.add(
                        PathPlanner.loadPath(
                                name,
                                Drivetrain.kMaxAutoVelocity,
                                Drivetrain.kMaxAutoAcceleration));
            }
        }
    }

    public static class Drivetrain {
        // Body Constants
        public static final double kWheelDiameter = Units.inchesToMeters(6);
        // Offset in meters from center of robot (also imu)
        public static final Translation2d kFrontLeftOffset =
                new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(-10.857));
        public static final Translation2d kRearLeftOffset =
                new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(-10.857));
        public static final Translation2d kFrontRightOffset =
                new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(10.857));
        public static final Translation2d kRearRightOffset =
                new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(10.857));
        /**
         * Theoretical max wheel speeds
         *
         * <p>Calculated from: https://x-engineer.org/calculate-wheel-vehicle-speed-engine-speed/
         * https://en.ans.wiki/105/how-to-convert-angular-velocity-in-revolutions-per-minute-rpm-to-linear-speed-in-meters-per-second-m-slash-s/
         *
         * <p>please kill me
         */
        public static final int kNEOFreeSpeedRPM = 5676;

        public static final double kMaxWheelSpeedRPM = 529.97;
        public static final double kMaxWheelSpeedMPS =
                ((2 * Math.PI * kWheelDiameter) / 60) * kMaxWheelSpeedRPM;

        // Joystick Constants
        public static final double kDeadband = 0.2;
        public static final double kSensitivity = 2.8;

        // Slew constants to allow for smoother control
        public static final double kXSlewRate = 0.5;
        public static final double kYSlewRate = 0.5;
        public static final double kZSlewRate = 0.2;

        // CAN IDs
        public static final int kFrontLeftId = 1;
        public static final int kRearLeftId = 2;
        public static final int kFrontRightId = 3;
        public static final int kRearRightId = 4;

        // Motor Type
        public static final CANSparkMax.MotorType kMotorType = CANSparkMax.MotorType.kBrushless;

        // autonomous constants
        public static final double kMaxAutoVelocity = 8.0; // m/s
        public static final double kMaxAutoAcceleration = 5.0; // m/s^2
        public static final PIDController kXPID = new PIDController(.02, 0, 0);
        public static final PIDController kYPID = new PIDController(.02, 0, 0);
        public static final ProfiledPIDController kThetaPID =
                new ProfiledPIDController(.02, 0, 0, new Constraints(6.28, 3.14));
        public static final PIDController kZPID = new PIDController(.018, 0, 0);

        // Gearbox Constants
        public static final double kGearRatio = 10.71; // 10.71:1 Toughboxes
        public static final double kRPMtoMPSConversionFactor =
                1.0 / (kGearRatio * (Math.PI * kWheelDiameter));
        public static final double kDistancePerPulse =
                1.0 / (42.0 /* Hall Effects CPR */ / kGearRatio * (Math.PI * kWheelDiameter));
    }

    public static class Arm {
        // Talon SRX CAN ID
        public static final int kId = 0;

        // speed as %; 1.00 == 100%, 0.50 == 50%, 0.00 == 0%
        public static final double kSpeed = 1.00;

        // Input:Output ratios
        public static final double kPlanetaryGearboxRatio = 81.0; // 81:1
        public static final double kGearAndChainRatio =
                12 / 60; // 12 teeth on gearbox, 60 teeth on arm

        // LED constants
        public static final int kLedLength = 60;
        public static final int kNumGradients = 4;
    }

    public static class Intake {
        // CAN ID
        public static final int kId = 5;

        // speed as %; 1.00 == 100%, 0.50 == 50%, 0.00 == 0%
        public static final double kSpeed = 1.00;

        // Red Line Motor Type
        public static final CANSparkMax.MotorType kMotorType = CANSparkMax.MotorType.kBrushed;
    }

    public static class Climber {
        // NEO 550 motor type
        public static final CANSparkMax.MotorType kMotorType = CANSparkMax.MotorType.kBrushed;

        // speed as %; 1.00 == 100%, 0.50 == 50%, 0.00 == 0%
        public static final double kSpeed = 0.75;
        // CANSparkMax CAN IDs
        public static final int kLId = 6;
        public static final int kRId = 7;
    }

    public static class LEDMatrix {
        public static final int kWidth = 15;
        public static final int kHeight = 10;
        public static final boolean kSerpentine = true;
        public static final int kPWMPort = 0;
        public static final Gif[] kGifs = new Gif[] {};
    }
}
