/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.kId;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Controls the arm of the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ArmSub implements Subsystem {

    final WPI_TalonSRX m_arm = new WPI_TalonSRX(kId);

    public final Command kDisable = new InstantCommand(() -> m_arm.stopMotor(), this);

    public final Command kUp =
            new RunCommand(() -> m_arm.set(1), this).withTimeout(2).andThen(kDisable);
    public final Command kDown =
            new RunCommand(() -> m_arm.set(-1), this).withTimeout(2).andThen(kDisable);

    public ArmSub() {
        m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        m_arm.setNeutralMode(NeutralMode.Brake);
        m_arm.setInverted(true);
    }

    public void disable() {
        CommandScheduler.getInstance().schedule(kDisable);
    }

    public boolean getFwdLimitSwitch() {
        return m_arm.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getRevLimitSwitch() {
        return m_arm.getSensorCollection().isRevLimitSwitchClosed();
    }
}
