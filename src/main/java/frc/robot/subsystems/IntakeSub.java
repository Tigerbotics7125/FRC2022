/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSub implements Subsystem {

    static final CANSparkMax m_intake = new CANSparkMax(kId, kMotorType);

    public IntakeSub() {}

    public void intake() {
        m_intake.setVoltage(12);
    }

    public void eject() {
        m_intake.setVoltage(-12);
    }
}
