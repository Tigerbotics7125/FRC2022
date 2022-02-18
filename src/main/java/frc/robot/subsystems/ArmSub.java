package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.constants.ArmConstants;

public class ArmSub extends TrapezoidProfileSubsystem {
  CANSparkMax m_motor = new CANSparkMax(ArmConstants.kArmId, ArmConstants.kArmMotorType);

  SparkMaxPIDController m_pidController = m_motor.getPIDController();

  ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts,
          ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad,
          ArmConstants.kAVoltSecondSquaredPerRad);

  public ArmSub() {
    super(
        new Constraints(
            ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared),
        ArmConstants.kArmOffsetRads);

    m_pidController.setP(ArmConstants.kP);
    m_pidController.setI(ArmConstants.kI);
    m_pidController.setD(ArmConstants.kD);
    m_pidController.setFF(ArmConstants.kFF);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    m_pidController.setReference(setpoint.position, CANSparkMax.ControlType.kPosition);
  }
}
