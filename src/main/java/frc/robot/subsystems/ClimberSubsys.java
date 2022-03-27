/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.Climber.kLId;
import static frc.robot.Constants.Climber.kMotorType;
import static frc.robot.Constants.Climber.kRId;
import static frc.robot.Constants.Climber.kSpeed;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the climber on each side of the robot Forward motor direction will winch the the
 * climber.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ClimberSubsys extends SubsystemBase {

    final CANSparkMax m_left = new CANSparkMax(kLId, kMotorType);
    final CANSparkMax m_right = new CANSparkMax(kRId, kMotorType);

    public ClimberSubsys() {
        m_left.setInverted(false);
        m_right.setInverted(true);
    }

    public void disable() {
        m_left.stopMotor();
        m_right.stopMotor();
    }

    public void winch() {
        m_left.set(1 * kSpeed);
        m_right.set(1 * kSpeed);
    }

    public void rappel() {
        m_left.set(-1 * kSpeed);
        m_right.set(-1 * kSpeed);
    }
}
