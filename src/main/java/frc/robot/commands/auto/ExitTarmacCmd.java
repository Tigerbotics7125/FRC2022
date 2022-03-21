/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsys;
import frc.robot.subsystems.DrivetrainSubsys;
import frc.robot.subsystems.IntakeSubsys;

/**
 * An autonomous routine which drives backwards to exit the tarmac
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ExitTarmacCmd extends SequentialCommandGroup {

    private DrivetrainSubsys mDrivetrain;
    private ArmSubsys mArm;
    private IntakeSubsys mIntake;

    public ExitTarmacCmd(DrivetrainSubsys drivetrain, ArmSubsys arm, IntakeSubsys intake) {
        // super(drivetrain, arm, intake);
        mDrivetrain = drivetrain;
        mArm = arm;
        mIntake = intake;
        addRequirements(mDrivetrain, mArm, mIntake);

        setName("Auto: Exit Tarmac");
        addCommands(
                new ParallelRaceGroup(
                        new RunCommand(() -> mDrivetrain.drive(-.5, 0, 0)), new WaitCommand(1)),
                new ParallelRaceGroup(
                        new RunCommand(mArm::lower, mArm),
                        new WaitUntilCommand(mArm::isDown),
                        new WaitCommand(2)));
    }
}
