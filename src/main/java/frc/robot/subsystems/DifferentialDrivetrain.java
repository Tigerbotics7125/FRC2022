package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Drive;

/**
 * The differential drivetrain of the robot, able to be simulated and work in autonomous.
 *
 * @author 7125 Tigerbotics - Jeffrey Morris
 */
public class DifferentialDrivetrain extends SubsystemBase {
  CANSparkMax m_left = new CANSparkMax(Constants.kDDLeftDeviceId, MotorType.kBrushless);
  CANSparkMax m_right = new CANSparkMax(Constants.kDDRightDeviceId, MotorType.kBrushless);

  RelativeEncoder m_leftEncoder = m_left.getEncoder();
  RelativeEncoder m_rightEncoder = m_right.getEncoder();

  PigeonIMU m_pigeon = new PigeonIMU(50);

  DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.kDDWheelBaseWidth);
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());

  // TODO: run sysid tools to find the values for our robot.
  // 42:00 - https://www.youtube.com/watch?v=wqJ4tY0u6IQ
  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0.268, 1.89, .243);

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

  public DifferentialDrivetrain() {
    m_left.setInverted(false);
    m_right.setInverted(true);

    if (!Robot.isReal()) {
      REVPhysicsSim.getInstance().addSparkMax(m_left, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_right, DCMotor.getNEO(1));
    }

    setDefaultCommand(new Drive(this));
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
    m_driveSim.setInputs(m_left.getAppliedOutput(), m_right.getAppliedOutput());

    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(wheelMetersToEncoderDistance(m_driveSim.getLeftPositionMeters()));
    m_rightEncoderSim.setDistance(
        wheelMetersToEncoderDistance(m_driveSim.getRightPositionMeters()));

    m_leftEncoderSim.setRate(wheelMPStoEncoderRPM(m_driveSim.getLeftVelocityMetersPerSecond()));
    m_rightEncoderSim.setRate(wheelMPStoEncoderRPM(m_driveSim.getRightVelocityMetersPerSecond()));

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
    double leftVelocityMPS;
    double rightVelocityMPS;

    if (Robot.isReal()) {
      leftVelocityMPS = encoderRPMToWheelMPS(m_leftEncoder.getVelocity());
      rightVelocityMPS = encoderRPMToWheelMPS(m_rightEncoder.getVelocity());
    } else {
      leftVelocityMPS = encoderRPMToWheelMPS(m_leftEncoderDummy.getRate());
      rightVelocityMPS = encoderRPMToWheelMPS(m_rightEncoderDummy.getRate());
    }

    return new DifferentialDriveWheelSpeeds(leftVelocityMPS, rightVelocityMPS);
  }

  @Override
  public void periodic() {

    double leftPositionMeters;
    double rightPositionMeters;

    if (Robot.isReal()) {
      leftPositionMeters = encoderDistanceToWheelMeters(m_leftEncoder.getPosition());
      rightPositionMeters = encoderDistanceToWheelMeters(m_rightEncoder.getPosition());
    } else {
      leftPositionMeters = encoderDistanceToWheelMeters(m_leftEncoderDummy.getDistance());
      rightPositionMeters = encoderDistanceToWheelMeters(m_rightEncoderDummy.getDistance());
    }

    m_odometry.update(getHeading(), leftPositionMeters, rightPositionMeters);

    Robot.m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void restartOdometry(Pose2d initialPose) {
    m_odometry = new DifferentialDriveOdometry(initialPose.getRotation(), initialPose);
  }

  public double encoderRPMToWheelMPS(double rpm) {
    return ((double) rpm) * Constants.kDDRPMToMPSConversionFactor;
  }

  public double wheelMPStoEncoderRPM(double mps) {
    return ((double) mps) / Constants.kDDRPMToMPSConversionFactor;
  }

  public double encoderDistanceToWheelMeters(double distance) {
    return ((double) distance) * Constants.kDDDistancePerPulse;
  }

  public double wheelMetersToEncoderDistance(double meters) {
    return ((double) meters) / Constants.kDDDistancePerPulse;
  }
}
