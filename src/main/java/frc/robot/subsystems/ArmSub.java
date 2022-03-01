/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.kAVoltSecondSquaredPerRad;
import static frc.robot.constants.ArmConstants.kArmOffsetRads;
import static frc.robot.constants.ArmConstants.kGVolts;
import static frc.robot.constants.ArmConstants.kGearAndChainRatio;
import static frc.robot.constants.ArmConstants.kId;
import static frc.robot.constants.ArmConstants.kPlanetaryGearboxRatio;
import static frc.robot.constants.ArmConstants.kSVolts;
import static frc.robot.constants.ArmConstants.kTrapezoidProfile;
import static frc.robot.constants.ArmConstants.kVVoltSecondPerRad;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class ArmSub extends TrapezoidProfileSubsystem {

    static final WPI_TalonSRX m_arm = new WPI_TalonSRX(kId);
    static final ArmFeedforward m_feedforward =
            new ArmFeedforward(kSVolts, kGVolts, kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

    public ArmSub() {
        super(kTrapezoidProfile, kArmOffsetRads);

        m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        m_arm.config_kP(0, 1);
        m_arm.config_kI(0, 0);
        m_arm.config_kD(0, 0);

        m_arm.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        m_arm.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        // setGoal(0); // lower arm

    }

    /**
     * gets the current measurement of the big driving gear from the encoder on minisim.
     *
     * @return the current measurement in radians
     */
    protected double getMeasurement() {
        return m_arm.getSelectedSensorPosition() / kPlanetaryGearboxRatio / kGearAndChainRatio;
    }

    @Override
    protected void useState(State state) {

        // guess the ouput for desired state
        double ff = m_feedforward.calculate(state.position, state.velocity);

        // set the output
        m_arm.set(ControlMode.Velocity, state.velocity + ff);
    }
}
