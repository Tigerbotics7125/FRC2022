/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MecanumDrivetrainSub;

public class ExitTarmac extends AutonomousCommand {

    private MecanumDrivetrainSub m_drivetrain = RobotContainer.kDrivetrain;

    public ExitTarmac() {
        /*
        * addCommands(
        * new ParallelRaceGroup(
        * new RunCommand(() -> m_drivetrain.drive(0, 1, 0)).withTimeout(.5),
        * new WaitCommand(1)));
        addCommands(
            new InstantCommand(() -> m_drivetrain.drive(0, -1, 0)).withTimeout(.5),
            new RunCommand(() -> RobotContainer.kIntake.eject()).withTimeout(1),
            new InstantCommand(() -> RobotContainer.kIntake.disable()));
            */
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
