/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.ClimberConstants.kLId;
import static frc.robot.constants.ClimberConstants.kMotorType;
import static frc.robot.constants.ClimberConstants.kRId;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

/**
 * Controls the climber on each side of the robot
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ClimberSub {

    final CANSparkMax m_left = new CANSparkMax(kLId, kMotorType);
    final CANSparkMax m_right = new CANSparkMax(kRId, kMotorType);

    public final Command kWinch =
            new ParallelRaceGroup(
                            new RunCommand(() -> m_left.set(1)),
                            new RunCommand(() -> m_right.set(1)),
                            new WaitCommand(.5),
                            // protects against climbing if our arm is not down.
                            new WaitUntilCommand(RobotContainer.kArm.isNotDown()))
                    .withName("Winch");

    public final Command kRepel =
            new ParallelRaceGroup(
                            new RunCommand(() -> m_left.set(-1)),
                            new RunCommand(() -> m_right.set(-1)),
                            new WaitCommand(.5),
                            // protects against climbing if our arm is not down.
                            new WaitUntilCommand(RobotContainer.kArm.isNotDown()))
                    .withName("Repel");

    public final Command kDisable =
            new ParallelCommandGroup(
                            new InstantCommand(() -> m_left.stopMotor()),
                            new InstantCommand(() -> m_right.stopMotor()))
                    .withName("disabled");

    public ClimberSub() {
        m_left.setInverted(true);
        m_right.setInverted(true);
    }
}
