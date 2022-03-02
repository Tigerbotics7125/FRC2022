/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.IntakeConstants.kId;
import static frc.robot.constants.IntakeConstants.kMotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSub implements Subsystem {

    static final CANSparkMax m_intake = new CANSparkMax(kId, kMotorType);
    static final SparkMaxPIDController m_pid = m_intake.getPIDController();

    public IntakeSub() {

        // m_intake.burnFlash();
    }

    public void intake() {
        m_pid.setReference(1, CANSparkMax.ControlType.kDutyCycle);
    }

    public void eject() {
        m_pid.setReference(-1, CANSparkMax.ControlType.kDutyCycle);
    }

    public void disable() {
        m_pid.setReference(0, CANSparkMax.ControlType.kDutyCycle);
    }
}
