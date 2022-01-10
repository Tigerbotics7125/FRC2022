package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  TalonSRX leftMaster = new TalonSRX(0);
  TalonSRX rightMaster = new TalonSRX(2);

  TalonSRX leftSlave = new TalonSRX(1);
  TalonSRX rightSlave = new TalonSRX(3);

  PigeonIMU pigeon = new PigeonIMU(0);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23));
  DifferentialDriveOdometry odomentry = new DifferentialDriveOdometry(getHeading());

  // TODO: run frc-characterization tools to find the values for our robot.
  // 42:00 - https://www.youtube.com/watch?v=wqJ4tY0u6IQ
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.268, 1.89, .243);

  // TODO: also run characterization for this too.
  // 47:00
  PIDController leftPIDController = new PIDController(9.95, 0, 0);
  PIDController rightPIDController = new PIDController(9.95, 0, 0);


  Pose2d pose;

  public Drivetrain() {

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

  }

  public Rotation2d getHeading() {

    // double[3] gyro;
    // pigeon.getRawGyro(gyro);
    // return Rotation2d.fromDegreens(gyro[2])

    /**
     * Gyroscopes in frc return positive values as you turn clockwise acording to
     * the unit circle this is backwards, thus we need to flip our gyro numbers.
     */
    return Rotation2d.fromDegrees(-pigeon.getFusedHeading());
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setOutput(double leftVoltage, double rightVoltage) {
    leftMaster.set(ControlMode.PercentOutput,  leftVoltage / 12);
    rightMaster.set(ControlMode.PercentOutput, rightVoltage / 12);
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSensorCollection().getPulseWidthVelocity(),
        rightMaster.getSensorCollection().getPulseWidthVelocity());
  }

  @Override
  public void periodic() {
    // TODO: fix with encoders, this feels wrong.
    pose = odomentry.update(getHeading(), leftMaster.getSensorCollection().getPulseWidthPosition(),
        rightMaster.getSensorCollection().getPulseWidthPosition());
  }
}
