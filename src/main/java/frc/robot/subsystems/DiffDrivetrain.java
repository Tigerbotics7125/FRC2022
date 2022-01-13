package frc.robot.subsystems;

import static frc.robot.Constants.kLeftDeviceId;
import static frc.robot.Constants.kRightDeviceId;
import static frc.robot.Constants.kWheelBaseWidthMeters;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DiffDrivetrain extends SubsystemBase {
  public CANSparkMax m_left = new CANSparkMax(kLeftDeviceId, MotorType.kBrushless);
  public CANSparkMax m_right = new CANSparkMax(kRightDeviceId, MotorType.kBrushless);

  RelativeEncoder m_leftEncoder = m_left.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 42);
  RelativeEncoder m_rightEncoder = m_right.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 42);

  PigeonIMU m_pigeon = new PigeonIMU(50);

  DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kWheelBaseWidthMeters);
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());

  // TODO: run sysid tools to find the values for our robot.
  // 42:00 - https://www.youtube.com/watch?v=wqJ4tY0u6IQ
  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0.268, 1.89, .243);

  // TODO: also run characterization for this too; setup in rev hardware stuff.
  SparkMaxPIDController m_leftSMPID = m_left.getPIDController();
  SparkMaxPIDController m_rightSMPID = m_right.getPIDController();
  PIDController m_leftPIDController = new PIDController(m_leftSMPID.getP(), m_leftSMPID.getI(), m_leftSMPID.getD());
  PIDController m_rightPIDController = new PIDController(m_rightSMPID.getP(), m_rightSMPID.getI(), m_rightSMPID.getD());

  // Simulation
  DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kSingleNEOPerSide,
      KitbotGearing.k10p71,
      KitbotWheelSize.kSixInch, VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  Encoder m_leftEncoderFake = new Encoder(0, 1, false);
  Encoder m_rightEncoderFake = new Encoder(2, 3, false);
  EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoderFake);
  EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoderFake);
  BasePigeonSimCollection m_pigeonSim = new BasePigeonSimCollection(m_pigeon, true);
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  public DiffDrivetrain() {
    m_left.setInverted(false);
    m_right.setInverted(true);
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(m_left.getAppliedOutput(), m_right.getAppliedOutput());
    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_pigeonSim.setRawHeading(-m_driveSim.getHeading().getDegrees());

  }

  public Rotation2d getHeading() {
    /**
     * Gyroscopes in frc return positive values as you turn clockwise acording to
     * the unit circle this is backwards, thus we need to flip our gyro numbers.
     */
    return Rotation2d.fromDegrees(-m_pigeon.getFusedHeading());
  }

  public SimpleMotorFeedforward getFeedForward() {
    return m_feedForward;
  }

  public PIDController getLeftPIDController() {
    return m_leftPIDController;
  }

  public PIDController getRightPIDController() {
    return m_rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setOutput(double leftVoltage, double rightVoltage) {
    m_left.setVoltage(leftVoltage / 12);
    m_right.setVoltage(rightVoltage / 12);
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    if (Robot.isReal()) {
      return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),
          m_rightEncoder.getVelocity());
    } else {
      return new DifferentialDriveWheelSpeeds(m_leftEncoderFake.getRate(), m_rightEncoderFake.getRate());
    }
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      m_odometry.update(getHeading(), m_leftEncoder.getPosition(),
          m_rightEncoder.getPosition());
    } else {
      m_odometry.update(getHeading(), m_leftEncoderFake.getDistance(), m_rightEncoderFake.getDistance());
    }

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    Robot.kField.setRobotPose(m_odometry.getPoseMeters());
  }

  public void restartOdometry(Pose2d initialPose) {
    m_odometry = new DifferentialDriveOdometry(initialPose.getRotation(), initialPose);
  }
}
