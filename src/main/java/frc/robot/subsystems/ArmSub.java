/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.kId;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ArmSub implements Subsystem {

    static final WPI_TalonSRX m_arm = new WPI_TalonSRX(kId);

    public ArmSub() {
        m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        m_arm.config_kP(0, 1);
        m_arm.config_kI(0, 0);
        m_arm.config_kD(0, 0);

        setDown(); // lower arm as soon as match starts
    }

    public void setUp() {
        m_arm.set(ControlMode.PercentOutput, -1);
    }

    public void setDown() {
        m_arm.set(ControlMode.PercentOutput, 1);
    }

    public void disable() {
        m_arm.stopMotor();
    }

    public Command getRaiseEjectLowerCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setUp()),
                new WaitCommand(1.25),
                new ParallelRaceGroup(
                        new RunCommand(() -> RobotContainer.kDrivetrain.drive(0, .75, 0)),
                        new WaitCommand(1)),
                new InstantCommand(() -> RobotContainer.kIntake.eject()),
                new WaitCommand(.5),
                new InstantCommand(() -> RobotContainer.kIntake.disable()),
                new ParallelRaceGroup(
                        new RunCommand(() -> RobotContainer.kDrivetrain.drive(0, -.75, 0)),
                        new WaitCommand(1)),
                new InstantCommand(() -> setDown()));
    }
}
