/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.kId;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Controls the arm of the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ArmSub extends SubsystemBase {

    final WPI_TalonSRX m_arm = new WPI_TalonSRX(kId);

    public final Command kDisable =
            new RunCommand(() -> m_arm.stopMotor(), this) {
                @Override
                public String getName() {
                    return "disabled";
                }
            };

    public final Command kUp =
            new SequentialCommandGroup(
                    new ParallelRaceGroup(
                            new RunCommand(() -> m_arm.set(1)),
                            new WaitUntilCommand(() -> getFwdLimitSwitch()),
                            new WaitCommand(2)),
                    new InstantCommand(() -> m_arm.stopMotor())) {
                @Override
                public String getName() {
                    return "Arm Up";
                }
            };

    public final Command kDown =
            new SequentialCommandGroup(
                    new ParallelRaceGroup(
                            new RunCommand(() -> m_arm.set(-1)),
                            new WaitUntilCommand(() -> getRevLimitSwitch()),
                            new WaitCommand(2)),
                    new InstantCommand(() -> m_arm.stopMotor())) {
                @Override
                public String getName() {
                    return "Arm Down";
                }
            };

    public ArmSub() {
        setDefaultCommand(kDisable);
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

    public BooleanSupplier isDown() {
        return () -> getRevLimitSwitch();
    }

    public BooleanSupplier isNotDown() {
        return () -> !getRevLimitSwitch();
    }

    public BooleanSupplier isUp() {
        return () -> getFwdLimitSwitch();
    }

    public BooleanSupplier isNotUp() {
        return () -> !getFwdLimitSwitch();
    }
}
