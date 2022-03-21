/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsys;
import frc.robot.subsystems.DrivetrainSubsys;
import frc.robot.subsystems.IntakeSubsys;

/**
 * A autonomous routine which does nothing, a placeholder for the auto selector.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class NoOpCmd extends SequentialCommandGroup {

    private DrivetrainSubsys mDrivetrain;
    private ArmSubsys mArm;
    private IntakeSubsys mIntake;

    public NoOpCmd(DrivetrainSubsys drivetrain, ArmSubsys arm, IntakeSubsys intake) {
        mDrivetrain = drivetrain;
        mArm = arm;
        mIntake = intake;
        addRequirements(mDrivetrain, mArm, mIntake);

        setName("Auto: No Op");
        addCommands(
                new ParallelCommandGroup(
                        new RunCommand(mDrivetrain::disable, mDrivetrain),
                        new RunCommand(mArm::disable, mArm),
                        new RunCommand(mIntake::disable, mIntake)));
    }
}
