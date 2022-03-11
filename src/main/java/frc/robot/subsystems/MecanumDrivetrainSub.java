/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.MecanumDrivetrainConstants.kDistancePerPulse;
import static frc.robot.constants.MecanumDrivetrainConstants.kFrontLeftId;
import static frc.robot.constants.MecanumDrivetrainConstants.kFrontLeftOffset;
import static frc.robot.constants.MecanumDrivetrainConstants.kFrontRightId;
import static frc.robot.constants.MecanumDrivetrainConstants.kFrontRightOffset;
import static frc.robot.constants.MecanumDrivetrainConstants.kMotorType;
import static frc.robot.constants.MecanumDrivetrainConstants.kRPMtoMPSConversionFactor;
import static frc.robot.constants.MecanumDrivetrainConstants.kRearLeftId;
import static frc.robot.constants.MecanumDrivetrainConstants.kRearLeftOffset;
import static frc.robot.constants.MecanumDrivetrainConstants.kRearRightId;
import static frc.robot.constants.MecanumDrivetrainConstants.kRearRightOffset;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Gamepads;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.tigerlib.Util;

/**
 * Controls the mecanum drivetrain of the robot
 *
 * @author 7125 Tigerbotics - Jeffrey Morris
 */
public class MecanumDrivetrainSub extends SubsystemBase {

    // Motors, PID controllers, and encoders
    final CANSparkMax m_frontLeft = new CANSparkMax(kFrontLeftId, kMotorType);
    final CANSparkMax m_rearLeft = new CANSparkMax(kRearLeftId, kMotorType);
    final CANSparkMax m_frontRight = new CANSparkMax(kFrontRightId, kMotorType);
    final CANSparkMax m_rearRight = new CANSparkMax(kRearRightId, kMotorType);

    final SparkMaxPIDController m_frontLeftPID = m_frontLeft.getPIDController();
    final SparkMaxPIDController m_rearLeftPID = m_rearLeft.getPIDController();
    final SparkMaxPIDController m_frontRightPID = m_frontRight.getPIDController();
    final SparkMaxPIDController m_rearRightPID = m_rearRight.getPIDController();

    final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
    final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
    final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
    final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

    // IMU, kinematics, and odometry
    final WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.kPigeonId);

    final MecanumDriveKinematics m_kinematics =
            new MecanumDriveKinematics(
                    kFrontLeftOffset, kFrontRightOffset, kRearLeftOffset, kRearRightOffset);

    final MecanumDriveOdometry m_odometry =
            new MecanumDriveOdometry(m_kinematics, new Rotation2d());

    // Variables used for different driving techniques
    boolean m_turning;
    boolean m_fieldOriented;
    boolean m_fieldOffset;

    // Commands used to operate the drivetrain
    public final Command kDisable =
            new ParallelCommandGroup(
                    new InstantCommand(() -> m_frontLeft.disable()),
                    new InstantCommand(() -> m_rearLeft.disable()),
                    new InstantCommand(() -> m_frontRight.disable()),
                    new InstantCommand(() -> m_rearRight.disable())) {
                @Override
                public String getName() {
                    return "disabled";
                }
            };

    public final Command kDriveWJoystick =
            new PerpetualCommand(new RunCommand(() -> drive(), this)) {
                @Override
                public String getName() {
                    return "drive w/ stick";
                }
            };

    public MecanumDrivetrainSub() {
        // set default command so we actually drive.
        setDefaultCommand(kDriveWJoystick);

        // set default driving options
        setTurning(true);
        setFieldOriented(false);

        // invert right side because motors backwards.
        m_frontLeft.setInverted(false);
        m_rearLeft.setInverted(false);
        m_frontRight.setInverted(true);
        m_rearRight.setInverted(true);

        m_frontLeftPID.setP(5e-5);
        m_rearLeftPID.setP(5e-5);
        m_frontRightPID.setP(5e-5);
        m_rearRightPID.setP(5e-5);

        // changes encoder distance from encoder ticks to meters
        m_frontLeftEncoder.setPositionConversionFactor(kDistancePerPulse);
        m_rearLeftEncoder.setPositionConversionFactor(kDistancePerPulse);
        m_frontRightEncoder.setPositionConversionFactor(kDistancePerPulse);
        m_rearRightEncoder.setPositionConversionFactor(kDistancePerPulse);

        // changes encoder velocity from rotations per minute to meters per second
        m_frontLeftEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        m_rearLeftEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        m_frontRightEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        m_rearRightEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);

        // make sure stuff starts on 0
        m_frontLeftEncoder.setPosition(0.0);
        m_rearLeftEncoder.setPosition(0.0);
        m_frontRightEncoder.setPosition(0.0);
        m_rearRightEncoder.setPosition(0.0);
        m_pigeon.setFusedHeading(0.0);

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_frontLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_rearLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_frontRight, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_rearRight, DCMotor.getNEO(1));
        }
    }

    /** general periodic updates. */
    @Override
    public void periodic() {
        // setSimHeading(HolonomicTestPath.getInstance().m_thetaPID.getSetpoint().position);
        m_odometry.update(getHeading(), getSpeeds());
    }

    /** update sparkmaxs during sim */
    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    /** reset the odometry of the drivetrain */
    public void resetOdometry(final Pose2d pose) {
        m_odometry.resetPosition(pose, getHeading());
    }

    /** enables or disables turning */
    public void setTurning(boolean turning) {
        m_turning = turning;
    }

    /** enables or disables field oriented driving */
    public void setFieldOriented(boolean fieldOriented) {
        m_fieldOriented = fieldOriented;
    }

    /** sets the heading of the robot */
    public void setHeading(Rotation2d heading) {
        m_pigeon.setFusedHeading(heading.getDegrees());
    }

    public void toggleFieldOffset() {
        m_fieldOffset = !m_fieldOffset;
    }

    public void toggleFieldOriented() {
        m_fieldOriented = !m_fieldOriented;
    }

    /**
     * Sets the drivetrain to move as per the given speeds.
     *
     * @param targetSpeeds The input speeds.
     */
    public void setSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {

        m_frontLeftPID.setReference(targetSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
        m_rearLeftPID.setReference(targetSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
        m_frontRightPID.setReference(targetSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
        m_rearRightPID.setReference(targetSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
    }

    /** drives with joysticks, converts to velocity then passes to feedforward. */
    public void drive() {
        double xSpeed = Gamepads.getRobotXInputSpeed();
        double ySpeed = Gamepads.getRobotYInputSpeed();
        double zSpeed = Gamepads.getRobotZInputSpeed();

        WheelSpeeds targetSpeeds =
                MecanumDrive.driveCartesianIK(
                        ySpeed,
                        xSpeed,
                        m_turning ? zSpeed : 0.0,
                        m_fieldOriented ? getHeading().getDegrees() : 0.0);

        m_frontLeftPID.setReference(targetSpeeds.frontLeft, ControlType.kDutyCycle);
        m_rearLeftPID.setReference(targetSpeeds.rearLeft, ControlType.kDutyCycle);
        m_frontRightPID.setReference(targetSpeeds.frontRight, ControlType.kDutyCycle);
        m_rearRightPID.setReference(targetSpeeds.rearRight, ControlType.kDutyCycle);
    }

    /**
     * Drives the robot based on manual input
     *
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Right is positive.
     * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Forward is positive.
     * @param zSpeed The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        xSpeed = Util.clamp(xSpeed, -1, 1);
        ySpeed = Util.clamp(ySpeed, -1, 1);
        zSpeed = Util.clamp(zSpeed, -1, 1);

        WheelSpeeds targetSpeeds = MecanumDrive.driveCartesianIK(ySpeed, xSpeed, zSpeed, 0.0);

        m_frontLeftPID.setReference(targetSpeeds.frontLeft, ControlType.kDutyCycle);
        m_rearLeftPID.setReference(targetSpeeds.rearLeft, ControlType.kDutyCycle);
        m_frontRightPID.setReference(targetSpeeds.frontRight, ControlType.kDutyCycle);
        m_rearRightPID.setReference(targetSpeeds.rearRight, ControlType.kDutyCycle);
    }

    /** Disables all motor output */
    public void disable() {
        m_frontLeft.disable();
        m_rearLeft.disable();
        m_frontRight.disable();
        m_rearRight.disable();
    }

    /** @return The current velocity of the robot. */
    public MecanumDriveWheelSpeeds getSpeeds() {
        return new MecanumDriveWheelSpeeds(
                m_frontLeftEncoder.getVelocity(),
                m_rearLeftEncoder.getVelocity(),
                m_frontRightEncoder.getVelocity(),
                m_rearRightEncoder.getVelocity());
    }

    /** @return If turning is enabled. */
    public boolean getTurning() {
        return m_turning;
    }

    /** @return If field oriented driving is enabled. */
    public boolean getFieldOriented() {
        return m_fieldOriented;
    }

    /** @return The drivetrains kinematics. */
    public MecanumDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /** @return the current heading of the robot */
    public Rotation2d getHeading() {
        // pigeon headings are already +CCW, no need to negate
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading() + (m_fieldOffset ? 180 : 0));
    }

    /** @returns the current position of the robot. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
}
