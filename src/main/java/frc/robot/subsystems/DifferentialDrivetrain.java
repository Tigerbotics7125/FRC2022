package frc.robot.subsystems;

import static frc.robot.Constants.kLeftDeviceId;
import static frc.robot.Constants.kRightDeviceId;
import static frc.robot.Constants.kWheelBaseWidthMeters;
import static frc.robot.Constants.kWheelDiameter;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.Drive;

public class DifferentialDrivetrain extends SubsystemBase {
  CANSparkMax m_left = new CANSparkMax(kLeftDeviceId, MotorType.kBrushless);
  CANSparkMax m_right = new CANSparkMax(kRightDeviceId, MotorType.kBrushless);

  RelativeEncoder m_leftEncoder =
      m_left.getEncoder();
  RelativeEncoder m_rightEncoder =
      m_right.getEncoder();

  PigeonIMU m_pigeon = new PigeonIMU(50);

  DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kWheelBaseWidthMeters);
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());

  // TODO: run sysid tools to find the values for our robot.
  // 42:00 - https://www.youtube.com/watch?v=wqJ4tY0u6IQ
  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0.268, 1.89, .243);

  // TODO: also run characterization for this too; setup in rev hardware stuff.
  SparkMaxPIDController m_leftSMPID = m_left.getPIDController();
  SparkMaxPIDController m_rightSMPID = m_right.getPIDController();
  PIDController m_leftPIDController =
      new PIDController(m_leftSMPID.getP(), m_leftSMPID.getI(), m_leftSMPID.getD());
  PIDController m_rightPIDController =
      new PIDController(m_rightSMPID.getP(), m_rightSMPID.getI(), m_rightSMPID.getD());

  // Simulation
  DifferentialDrivetrainSim m_driveSim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kSingleNEOPerSide,
          KitbotGearing.k10p71,
          KitbotWheelSize.kSixInch,
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  Encoder m_leftEncoderDummy = new Encoder(0, 1);
  Encoder m_rightEncoderDummy = new Encoder(2, 3);
  ADXRS450_Gyro m_gyroDummy = new ADXRS450_Gyro();
  EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoderDummy);
  EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoderDummy);
  ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyroDummy);

  // NT
  NetworkTableEntry m_xEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  NetworkTableEntry m_leftVelocityEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("leftVelocity");
  NetworkTableEntry m_leftPositionEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("leftPosition");
  NetworkTableEntry m_leftVoltage =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("leftVoltage");
  NetworkTableEntry m_rightVelocityEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("rightVelocity");
  NetworkTableEntry m_rightPositionEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("rightPosition");
  NetworkTableEntry m_rightVoltage =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("rightVoltage");
  NetworkTableEntry m_headingEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Heading");
  NetworkTableEntry m_odometryEntry =
      NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Odometry");

  public DifferentialDrivetrain() {
    m_left.setInverted(false);
    m_right.setInverted(true);
    m_leftEncoder.setPositionConversionFactor(42 / 10.71 * (Math.PI * Units.inchesToMeters(6)));
    m_rightEncoder.setPositionConversionFactor(42 / 10.71 * (Math.PI * Units.inchesToMeters(6)));

    if (!Robot.isReal()) {
      REVPhysicsSim.getInstance().addSparkMax(m_left, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_right, DCMotor.getNEO(1));
      m_leftEncoderDummy.setDistancePerPulse(42 / 10.71 * (Math.PI * Units.inchesToMeters(6)));
      m_rightEncoderDummy.setDistancePerPulse(42 / 10.71 * (Math.PI * Units.inchesToMeters(6)));
    }

    setDefaultCommand(new Drive(this));
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
    m_driveSim.setInputs(m_left.getAppliedOutput(), m_right.getAppliedOutput());
    
    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_rightEncoderSim.setDistance(m_driveSim.getLeftVelocityMetersPerSecond());
    m_leftEncoderSim.setRate(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());


  }

  public Rotation2d getHeading() {
    /**
     * Gyroscopes in frc return positive values as you turn clockwise acording to the unit circle
     * this is backwards, thus we need to flip our gyro numbers.
     */
    if (Robot.isReal()) {
      return Rotation2d.fromDegrees(-m_pigeon.getFusedHeading());
    } else {
      return m_gyroDummy.getRotation2d();
    }
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
    m_left.setVoltage(leftVoltage);
    m_right.setVoltage(rightVoltage);
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {

    return new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      m_odometry.update(getHeading(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    } else {
      m_odometry.update(
          getHeading(), m_leftEncoderDummy.getDistance(), m_rightEncoderDummy.getDistance());
    }

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
    m_leftVelocityEntry.setNumber(m_leftEncoder.getVelocity());
    m_leftPositionEntry.setNumber(m_leftEncoder.getPosition());
    m_leftVoltage.setNumber(m_left.getAppliedOutput());
    m_rightVelocityEntry.setNumber(m_rightEncoder.getVelocity());
    m_rightPositionEntry.setNumber(m_rightEncoder.getPosition());
    m_rightVoltage.setNumber(m_right.getAppliedOutput());
    m_headingEntry.setNumber(getHeading().getDegrees());

    Robot.m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void restartOdometry(Pose2d initialPose) {
    m_odometry = new DifferentialDriveOdometry(initialPose.getRotation(), initialPose);
  }
}
