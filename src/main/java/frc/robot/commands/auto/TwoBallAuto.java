/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import static frc.robot.constants.MecanumDrivetrainConstants.*;

import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.constants.AutonomousTrajectories;
import frc.robot.subsystems.MecanumDrivetrainSub;

public class TwoBallAuto extends SequentialCommandGroup {

    private MecanumDrivetrainSub m_drivetrain = RobotContainer.kDrivetrain;

    private Command m_mecCommand =
            new PPMecanumControllerCommand(
                    AutonomousTrajectories.kTwoBallAuto[0],
                    m_drivetrain::getPose,
                    m_drivetrain.getKinematics(),
                    kXPID,
                    kYPID,
                    kThetaPID,
                    kMaxSpeed,
                    m_drivetrain::setSpeeds,
                    m_drivetrain);
}
