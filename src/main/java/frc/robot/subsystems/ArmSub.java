/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.kId;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSub implements Subsystem {

    static final WPI_TalonSRX m_arm = new WPI_TalonSRX(kId);

    public ArmSub() {
        m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        m_arm.config_kP(0, 1);
        m_arm.config_kI(0, 0);
        m_arm.config_kD(0, 0);
        // setGoal(0); // lower arm

    }

    public void setUp() {
        m_arm.set(ControlMode.PercentOutput, -1);
    }

    public void setDown() {
        m_arm.set(ControlMode.PercentOutput, 1);
    }
}
