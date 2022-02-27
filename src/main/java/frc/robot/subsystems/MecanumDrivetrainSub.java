/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.MecanumDrivetrainConstants.*;

import com.ctre.phoenix.sensors.PigeonIMU;
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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DashboardManager;
import frc.robot.Robot;
import frc.robot.commands.MecanumDrivetrainCom;
import frc.robot.constants.Constants;

/**
 * The mecanum drivetrain of the robot, able to be simulated and work inf autonomous.
 *
 * @author 7125 Tigerbotics - Jeffrey Morris
 */
public class MecanumDrivetrainSub extends MecanumDrive implements Subsystem {

    // Motors, PID controllers, and encoders
    static final CANSparkMax m_frontLeft = new CANSparkMax(kFrontLeftId, kMotorType);
    static final CANSparkMax m_rearLeft = new CANSparkMax(kRearLeftId, kMotorType);
    static final CANSparkMax m_frontRight = new CANSparkMax(kFrontRightId, kMotorType);
    static final CANSparkMax m_rearRight = new CANSparkMax(kRearRightId, kMotorType);

    static final SparkMaxPIDController m_frontLeftPID = m_frontLeft.getPIDController();
    static final SparkMaxPIDController m_rearLeftPID = m_rearLeft.getPIDController();
    static final SparkMaxPIDController m_frontRightPID = m_frontRight.getPIDController();
    static final SparkMaxPIDController m_rearRightPID = m_rearRight.getPIDController();

    static final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
    static final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
    static final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
    static final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

    // IMU, kinematics, and odometry
    static final PigeonIMU m_pigeon = new PigeonIMU(Constants.kPigeonId);
    static final WPI_PigeonIMU m_simPigeon = new WPI_PigeonIMU(Constants.kPigeonId);

    static final MecanumDriveKinematics m_kinematics =
            new MecanumDriveKinematics(
                    kFrontLeftOffset, kFrontRightOffset, kRearLeftOffset, kRearRightOffset);

    static final MecanumDriveOdometry m_odometry =
            new MecanumDriveOdometry(m_kinematics, new Rotation2d());

    // Variables for dashboard
    static double m_frontLeftVelocitySetpoint = 0;
    static double m_rearLeftVelocitySetpoint = 0;
    static double m_frontRightVelocitySetpoint = 0;
    static double m_rearRightVelocitySetpoint = 0;

    public MecanumDrivetrainSub() {
        super(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight); // super constructor
        // set max output so we can go faster / dont go too fast, a bit oxymoronic.
        setMaxOutput(kMaxSpeed);

        // invert right side because motors backwards.
        m_frontLeft.setInverted(false);
        m_rearLeft.setInverted(true);
        m_frontRight.setInverted(false);
        m_rearRight.setInverted(true);

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

        // make sure encoders start on 0
        m_frontLeftEncoder.setPosition(0.0);
        m_rearLeftEncoder.setPosition(0.0);
        m_frontRightEncoder.setPosition(0.0);
        m_rearRightEncoder.setPosition(0.0);

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_frontLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_rearLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_frontRight, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_rearRight, DCMotor.getNEO(1));
        }

        // sets drive command when not in auto
        setDefaultCommand(new MecanumDrivetrainCom(this));
    }

    /** general periodic updates. */
    @Override
    public void periodic() {
        // setSimHeading(HolonomicTestPath.getInstance().m_thetaPID.getSetpoint().position);
        m_odometry.update(getHeading(), getSpeeds());
        DashboardManager.kField.setRobotPose(m_odometry.getPoseMeters());
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

    /** sets the drivetrain to move according to the input. */
    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        m_frontLeftVelocitySetpoint = speeds.frontLeftMetersPerSecond;
        m_rearLeftVelocitySetpoint = speeds.rearLeftMetersPerSecond;
        m_frontRightVelocitySetpoint = speeds.frontRightMetersPerSecond;
        m_rearRightVelocitySetpoint = speeds.rearRightMetersPerSecond;

        m_frontLeftPID.setReference(speeds.frontLeftMetersPerSecond, ControlType.kVelocity);
        m_rearLeftPID.setReference(speeds.rearLeftMetersPerSecond, ControlType.kVelocity);
        m_frontRightPID.setReference(speeds.frontRightMetersPerSecond, ControlType.kVelocity);
        m_rearRightPID.setReference(speeds.rearRightMetersPerSecond, ControlType.kVelocity);

        feed();
    }

    public void setSimHeading(double degrees) {
        m_simPigeon.setFusedHeading(degrees);
    }

    /** @returns the current velocity of the robot. */
    public MecanumDriveWheelSpeeds getSpeeds() {
        return new MecanumDriveWheelSpeeds(
                m_frontLeftEncoder.getVelocity(),
                m_rearLeftEncoder.getVelocity(),
                m_frontRightEncoder.getVelocity(),
                m_rearRightEncoder.getVelocity());
    }

    /** @returns the drivetrains kinematics */
    public MecanumDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * the heading but negative because of the unit circle vs gyro.
     *
     * @return the current heading of the robot
     */
    public Rotation2d getHeading() {
        if (Robot.isReal()) {
            return Rotation2d.fromDegrees(-m_pigeon.getFusedHeading());
        } else {
            return Rotation2d.fromDegrees(-m_simPigeon.getFusedHeading());
        }
    }

    /** @returns the current position of the robot. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
}
