package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DashboardManager;
import frc.robot.DashboardManager.Tab;
import frc.robot.Robot;
import frc.robot.commands.MecanumDrivetrainCom;
import frc.robot.constants.Constants;
import frc.robot.constants.MecanumDrivetrainConstants;

/**
 * The mecanum drivetrain of the robot, able to be simulated and work inf autonomous.
 *
 * @author 7125 Tigerbotics - Jeffrey Morris
 */
public class MecanumDrivetrainSub extends MecanumDrive implements Subsystem {

  // local constants to reduce clutter
  static final double kMaxSpeed = MecanumDrivetrainConstants.kMaxSpeed;
  static final double kMaxAngularSpeed = MecanumDrivetrainConstants.kMaxAngularSpeed;
  static final int kFrontLeftId = MecanumDrivetrainConstants.kFrontLeftId;
  static final int kRearLeftId = MecanumDrivetrainConstants.kRearLeftId;
  static final int kFrontRightId = MecanumDrivetrainConstants.kFrontRightId;
  static final int kRearRightId = MecanumDrivetrainConstants.kRearRightId;
  static final com.revrobotics.CANSparkMaxLowLevel.MotorType kMotorType =
      MecanumDrivetrainConstants.kMotorType;
  static final Translation2d kFrontLeftOffset = MecanumDrivetrainConstants.kFrontLeftOffset;
  static final Translation2d kRearLeftOffset = MecanumDrivetrainConstants.kRearLeftOffset;
  static final Translation2d kFrontRightOffset = MecanumDrivetrainConstants.kFrontRightOffset;
  static final Translation2d kRearRightOffset = MecanumDrivetrainConstants.kRearRightOffset;
  static final double kRPMtoMPSConversionFactor =
      MecanumDrivetrainConstants.kRPMtoMPSConversionFactor;
  static final double kDistancePerPulse = MecanumDrivetrainConstants.kDistancePerPulse;

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
    super(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

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

    // dashboard stuffz
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber(
            "Robot X Vel", () -> m_kinematics.toChassisSpeeds(getSpeeds()).vxMetersPerSecond);
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber(
            "Robot Y Vel", () -> m_kinematics.toChassisSpeeds(getSpeeds()).vyMetersPerSecond);
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber(
            "Robot Theta Vel",
            () -> m_kinematics.toChassisSpeeds(getSpeeds()).omegaRadiansPerSecond);
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot Rot Deg", () -> getHeading().getDegrees());
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot FL Vel", this::getFLVelocity);
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot RL Vel", this::getRLVelocity);
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot FR Vel", this::getFRVelocity);
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot RR Vel", this::getRRVelocity);
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber("Robot FL Vel Setpoint", this::getFLVelocitySetpoint);
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber("Robot RL Vel Setpoint", this::getRLVelocitySetpoint);
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber("Robot FR Vel Setpoint", this::getFRVelocitySetpoint);
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber("Robot RR Vel Setpoint", this::getRRVelocitySetpoint);

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
    DashboardManager.getField().setRobotPose(m_odometry.getPoseMeters());
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
  @Override
  public void driveCartesian(
      final double xJoystick, final double yJoystick, final double zJoystick) {
    double xSpeed = MathUtil.clamp(xJoystick, -1.0, 1.0) * MecanumDrivetrainConstants.kMaxSpeed;
    double ySpeed = MathUtil.clamp(yJoystick, -1.0, 1.0) * MecanumDrivetrainConstants.kMaxSpeed;
    double zSpeed =
        MathUtil.clamp(zJoystick, -1.0, 1.0) * MecanumDrivetrainConstants.kMaxAngularSpeed;
    final MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getHeading()));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
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
  }

  public void setSimHeading(double degrees) {
    m_simPigeon.setFusedHeading(degrees);
  }

  public double getFLVelocity() {
    return m_frontLeftEncoder.getVelocity();
  }

  public double getFLVelocitySetpoint() {
    return m_frontLeftVelocitySetpoint;
  }

  public double getRLVelocity() {
    return m_rearLeftEncoder.getVelocity();
  }

  public double getRLVelocitySetpoint() {
    return m_rearLeftVelocitySetpoint;
  }

  public double getFRVelocity() {
    return m_frontRightEncoder.getVelocity();
  }

  public double getFRVelocitySetpoint() {
    return m_frontRightVelocitySetpoint;
  }

  public double getRRVelocity() {
    return m_rearRightEncoder.getVelocity();
  }

  public double getRRVelocitySetpoint() {
    return m_rearRightVelocitySetpoint;
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
