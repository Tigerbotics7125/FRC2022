package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
public class MecanumDrivetrainSub extends SubsystemBase {

  // local constants to reduce clutter
  static final double kMaxSpeed = MecanumDrivetrainConstants.kMaxSpeed;
  static final double kMaxAngularSpeed = MecanumDrivetrainConstants.kMaxAngularSpeed;
  static final int kFrontLeftId = MecanumDrivetrainConstants.kFrontLeftId;
  static final int kRearLeftId = MecanumDrivetrainConstants.kRearLeftId;
  static final int kFrontRightId = MecanumDrivetrainConstants.kFrontRightId;
  static final int kRearRightId = MecanumDrivetrainConstants.kRearRightId;
  static final MotorType kMotorType = MecanumDrivetrainConstants.kMotorType;
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

  static final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          kFrontLeftOffset, kFrontRightOffset, kRearLeftOffset, kRearRightOffset);

  static final MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(m_kinematics, new Rotation2d());

  public MecanumDrivetrainSub() {
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
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot X Vel", this::getXVelocity);
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot Y Vel", this::getYVelocity);
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot Rot Deg", this.getHeading()::getDegrees);

    if (Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(m_frontLeft, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rearLeft, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_frontRight, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rearRight, DCMotor.getNEO(1));
    }

    // sets drive command when not in auto
    setDefaultCommand(new MecanumDrivetrainCom(this));
  }

  /**
   * the heading but negative because of the unit circle vs gyro.
   *
   * @return the current heading of the robot
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_pigeon.getFusedHeading());
  }

  /** @returns the current position of the robot. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** sets the drivetrain to move according to the input. */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    m_frontLeftPID.setReference(speeds.frontLeftMetersPerSecond, ControlType.kVelocity);
    m_rearLeftPID.setReference(speeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    m_frontRightPID.setReference(speeds.frontRightMetersPerSecond, ControlType.kVelocity);
    m_rearRightPID.setReference(speeds.rearRightMetersPerSecond, ControlType.kSmartVelocity);
  }

  /** sets the drivetrain to move according to the input. */
  public void drive(
      final double xSpeed, final double ySpeed, final double rot, final boolean fieldRelative) {
    final MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  /** @returns the current velocity of the robot. */
  public MecanumDriveWheelSpeeds getSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  /** general periodic updates. */
  @Override
  public void periodic() {
    m_odometry.update(getHeading(), getSpeeds());
    DashboardManager.getField().setRobotPose(m_odometry.getPoseMeters());
  }

  public double getXVelocity() {
    return m_kinematics.toChassisSpeeds(getSpeeds()).vxMetersPerSecond;
  }

  public double getYVelocity() {
    return m_kinematics.toChassisSpeeds(getSpeeds()).vyMetersPerSecond;
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public MecanumDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void resetOdometry(final Pose2d pose) {
    m_odometry.resetPosition(pose, getHeading());
  }
}
