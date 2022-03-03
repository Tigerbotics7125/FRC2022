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
import static frc.robot.constants.MecanumDrivetrainConstants.kMaxSpeed;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DashboardManager;
import frc.robot.Gamepads;
import frc.robot.Robot;
import frc.robot.commands.Drive;
import frc.robot.constants.Constants;

/**
 * The mecanum drivetrain of the robot, able to be simulated and work inf
 * autonomous.
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
    static final WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.kPigeonId);

    static final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
            kFrontLeftOffset, kFrontRightOffset, kRearLeftOffset, kRearRightOffset);

    static final MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d());

    // variables for this buttons to control
    static boolean m_turning = true;
    static boolean m_fieldOriented = true;

    // Variables for dashboard
    static double m_frontLeftVelocitySetpoint = 0;
    static double m_rearLeftVelocitySetpoint = 0;
    static double m_frontRightVelocitySetpoint = 0;
    static double m_rearRightVelocitySetpoint = 0;

    public MecanumDrivetrainSub() {
        super(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight); // super constructor
        // set max output so we can go faster / dont go too fast, a bit oxymoronic.
        setMaxOutput(kMaxSpeed);
        // set default command so we actually drive.
        setDefaultCommand(new Drive(this));

        // invert right side because motors backwards.
        m_frontLeft.setInverted(false);
        m_rearLeft.setInverted(false);
        m_frontRight.setInverted(true);
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

    /** sets the drivetrain to move according to the input. */
    public void setSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
        /*
         * MecanumDriveWheelSpeeds currentSpeeds = getSpeeds();
         * double now = Timer.getFPGATimestamp();
         * System.out.println("input Speeds:" + targetSpeeds.toString());
         * 
         * double flSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.frontLeftMetersPerSecond,
         * targetSpeeds.frontLeftMetersPerSecond,
         * now - m_lastTime);
         * double rlSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.rearLeftMetersPerSecond,
         * targetSpeeds.rearLeftMetersPerSecond,
         * now - m_lastTime);
         * double frSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.frontRightMetersPerSecond,
         * targetSpeeds.frontRightMetersPerSecond,
         * now - m_lastTime);
         * double rrSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.rearRightMetersPerSecond,
         * targetSpeeds.rearRightMetersPerSecond,
         * now - m_lastTime);
         * 
         * System.out.println("output Speeds:" + flSpeed + " " + rlSpeed + " " + frSpeed
         * + " " + rrSpeed);
         * 
         * m_frontLeftPID.setReference(flSpeed, ControlType.kDutyCycle);
         * m_rearLeftPID.setReference(rlSpeed, ControlType.kDutyCycle);
         * m_frontRightPID.setReference(frSpeed, ControlType.kDutyCycle);
         * m_rearRightPID.setReference(rrSpeed, ControlType.kDutyCycle);
         * 
         */

        m_frontLeftPID.setReference(targetSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
        m_rearLeftPID.setReference(targetSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
        m_frontRightPID.setReference(targetSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
        m_rearRightPID.setReference(targetSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);

        // m_lastTime = Timer.getFPGATimestamp();
        feed();
    }

    /** drives with joysticks, converts to velocity then passes to feedforward. */
    public void drive() {
        double xSpeed = Gamepads.getRobotXInputSpeed();
        double ySpeed = Gamepads.getRobotYInputSpeed();
        double zSpeed = Gamepads.getRobotZInputSpeed();

        WheelSpeeds targetSpeeds = MecanumDrive.driveCartesianIK(
                ySpeed,
                xSpeed,
                m_turning ? zSpeed : 0.0,
                m_fieldOriented ? getHeading().getDegrees() : 0.0);
        /*
         * MecanumDriveWheelSpeeds currentSpeeds = getSpeeds();
         * double now = Timer.getFPGATimestamp();
         * 
         * double flSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.frontLeftMetersPerSecond,
         * targetSpeeds.frontLeft * kMaxWheelSpeedMPS,
         * now - m_lastTime);
         * double rlSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.rearLeftMetersPerSecond,
         * targetSpeeds.rearLeft * kMaxWheelSpeedMPS,
         * now - m_lastTime);
         * double frSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.frontRightMetersPerSecond,
         * targetSpeeds.frontRight * kMaxWheelSpeedMPS,
         * now - m_lastTime);
         * double rrSpeed =
         * m_feedforward.calculate(
         * currentSpeeds.rearRightMetersPerSecond,
         * targetSpeeds.rearRight * kMaxWheelSpeedMPS,
         * now - m_lastTime);
         * 
         * m_frontLeftPID.setReference(flSpeed, ControlType.kDutyCycle);
         * m_rearLeftPID.setReference(rlSpeed, ControlType.kDutyCycle);
         * m_frontRightPID.setReference(frSpeed, ControlType.kDutyCycle);
         * m_rearRightPID.setReference(rrSpeed, ControlType.kDutyCycle);
         * 
         * m_lastTime = Timer.getFPGATimestamp();
         */
        m_frontLeftPID.setReference(targetSpeeds.frontLeft, ControlType.kDutyCycle);
        m_rearLeftPID.setReference(targetSpeeds.rearLeft, ControlType.kDutyCycle);
        m_frontRightPID.setReference(targetSpeeds.frontRight, ControlType.kDutyCycle);
        m_rearRightPID.setReference(targetSpeeds.rearRight, ControlType.kDutyCycle);

        feed();
    }

    public void drive(double ySpeed, double xSpeed, double zSpeed) {
        WheelSpeeds targetSpeeds = MecanumDrive.driveCartesianIK(
                ySpeed,
                xSpeed,
                zSpeed,
                0.0);

        m_frontLeftPID.setReference(targetSpeeds.frontLeft, ControlType.kDutyCycle);
        m_rearLeftPID.setReference(targetSpeeds.rearLeft, ControlType.kDutyCycle);
        m_frontRightPID.setReference(targetSpeeds.frontRight, ControlType.kDutyCycle);
        m_rearRightPID.setReference(targetSpeeds.rearRight, ControlType.kDutyCycle);

        feed();
    }

    /** @returns the current velocity of the robot. */
    public MecanumDriveWheelSpeeds getSpeeds() {
        return new MecanumDriveWheelSpeeds(
                m_frontLeftEncoder.getVelocity(),
                m_rearLeftEncoder.getVelocity(),
                m_frontRightEncoder.getVelocity(),
                m_rearRightEncoder.getVelocity());
    }

    public boolean getTurning() {
        return m_turning;
    }

    public boolean getFieldOriented() {
        return m_fieldOriented;
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
        // pigeon headings are already +CCW, no need to negate
        return Rotation2d.fromDegrees(
                m_pigeon.getFusedHeading()
                        + (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180));
    }

    /** @returns the current position of the robot. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
}
