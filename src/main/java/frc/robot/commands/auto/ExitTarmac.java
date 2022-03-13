/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MecanumDrivetrainSub;

public class ExitTarmac extends AutonomousCommand {

    private MecanumDrivetrainSub m_drivetrain = RobotContainer.kDrivetrain;

    public ExitTarmac() {
        addCommands(
                new ParallelRaceGroup(
                        new RunCommand(() -> m_drivetrain.drive(-.5, 0, 0)), new WaitCommand(1)),
                RobotContainer.kArm.kDown);
    }

    @Override
    public Pose2d getInitialPose() {
        // TODO Auto-generated method stub
        return new Pose2d();
    }

    @Override
    public void preview() {
        // TODO Auto-generated method stub

    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
