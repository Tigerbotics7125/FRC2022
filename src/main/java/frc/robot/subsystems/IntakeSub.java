/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.IntakeConstants.kId;
import static frc.robot.constants.IntakeConstants.kMotorType;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Controls the intake of the robot
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class IntakeSub implements Subsystem {

    final CANSparkMax m_intake = new CANSparkMax(kId, kMotorType);

    public final Command kIntake = new RunCommand(() -> m_intake.set(1), this);
    public final Command kEject = new RunCommand(() -> m_intake.set(-1), this);
    public final Command kDisable = new InstantCommand(() -> m_intake.set(0), this);

    public void disable() {
        CommandScheduler.getInstance().schedule(kDisable);
    }
}
