package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The mecanum drivetrain of the robot, able to be simulated and work in autonomous.
 *
 * @author 7125 Tigerbotics - Jeffrey Morris
 */
public class MecanumDrivetrain extends SubsystemBase {
  CANSparkMax m_leftFront = new CANSparkMax(Constants.kMDLeftFrontId, Constants.kMDMotorType);
  CANSparkMax m_leftRear = new CANSparkMax(Constants.kMDLeftRearId, Constants.kMDMotorType);
  CANSparkMax m_rightFront = new CANSparkMax(Constants.kMDRightFrontId, Constants.kMDMotorType);
  CANSparkMax m_rightRear = new CANSparkMax(Constants.kMDRightRearId, Constants.kMDMotorType);

  RelativeEncoder m_leftFrontEncoder = m_leftFront.getEncoder();
  RelativeEncoder m_leftRearEncoder = m_leftRear.getEncoder();
  RelativeEncoder m_rightFrontEncoder = m_rightFront.getEncoder();
  RelativeEncoder m_rightRearEncoder = m_rightRear.getEncoder();

  PigeonIMU m_pigeon = new PigeonIMU(Constants.kPigeonId);

  MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          Constants.kMDLeftFrontOffset,
          Constants.kMDRightFrontOffset,
          Constants.kMDLeftRearOffset,
          Constants.kMDRightRearOffset);

  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d());

  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.268, 1.89, .243);

  SparkMaxPIDController m_leftFrontPID = m_leftFront.getPIDController();
  SparkMaxPIDController m_leftRearPID = m_leftRear.getPIDController();
  SparkMaxPIDController m_rightFrontPID = m_rightFront.getPIDController();
  SparkMaxPIDController m_rightRearPID = m_rightRear.getPIDController();

  // simulation

  public MecanumDrivetrain() {
    m_leftFront.setInverted(false);
    m_leftRear.setInverted(false);
    m_rightFront.setInverted(true);
    m_rightRear.setInverted(true);

    // setDefaultCommand(new MecanumDrive(this));
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_pigeon.getFusedHeading());
  }

  public SimpleMotorFeedforward getFeedFeedforward() {
    return m_feedforward;
  }

  public PIDController getLeftFrontPIDController() {
    return new PIDController(m_leftFrontPID.getP(), m_leftFrontPID.getI(), m_leftFrontPID.getD());
  }

  public PIDController getLeftRearPIDController() {
    return new PIDController(m_leftRearPID.getP(), m_leftRearPID.getI(), m_leftRearPID.getD());
  }

  public PIDController getRightFrontPIDController() {
    return new PIDController(
        m_rightFrontPID.getP(), m_rightFrontPID.getI(), m_rightFrontPID.getD());
  }

  public PIDController getRightRearPIDController() {
    return new PIDController(m_rightRearPID.getP(), m_rightRearPID.getI(), m_rightRearPID.getD());
  }

  public MecanumDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setOutput(
      double leftFrontVolts, double leftRearVolts, double rightFrontVolts, double rightRearVolts) {
    m_leftFront.setVoltage(leftFrontVolts);
    m_leftRear.setVoltage(leftRearVolts);
    m_rightFront.setVoltage(rightFrontVolts);
    m_rightRear.setVoltage(rightRearVolts);
  }

  public MecanumDriveWheelSpeeds getSpeeds() {
    return new MecanumDriveWheelSpeeds(
        rpmToMps(m_leftFrontEncoder.getVelocity()),
        rpmToMps(m_leftRearEncoder.getVelocity()),
        rpmToMps(m_rightFrontEncoder.getVelocity()),
        rpmToMps(m_rightRearEncoder.getVelocity()));
  }

  @Override
  public void periodic() {
    m_odometry.update(getHeading(), getSpeeds());
  }

  public double rpmToMps(double rpm) {
    return ((double) rpm) * Constants.kMDRPMtoMPSConversionFactor;
  }

  public double distanceToMeters(double distance) {
    return ((double) distance) * Constants.kMDDistancePerPulse;
  }
}
