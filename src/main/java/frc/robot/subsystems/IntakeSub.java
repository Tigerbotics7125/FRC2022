/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.IntakeConstants.kId;
import static frc.robot.constants.IntakeConstants.kMotorType;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Controls the intake of the robot
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class IntakeSub implements Subsystem {

    static final CANSparkMax m_intake = new CANSparkMax(kId, kMotorType);

    public IntakeSub() {}

    public void intake() {
        m_intake.set(.8);
    }

    public void eject() {
        m_intake.set(-.8);
    }

    public void disable() {
        m_intake.set(0);
    }
}
