/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import static frc.robot.constants.MecanumDrivetrainConstants.kMaxSpeed;
import static frc.robot.constants.MecanumDrivetrainConstants.kThetaPID;
import static frc.robot.constants.MecanumDrivetrainConstants.kXPID;
import static frc.robot.constants.MecanumDrivetrainConstants.kYPID;

import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.constants.AutonomousTrajectories;
import frc.robot.subsystems.MecanumDrivetrainSub;
import frc.tigerlib.command.AutonomousCommand;

/**
 * A simple test path to test autonomous driving our mecanum drivetrain.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class HolonomicTestPath extends SequentialCommandGroup implements AutonomousCommand {

    private MecanumDrivetrainSub m_drivetrain = RobotContainer.kDrivetrain;

    private Command m_mecConCom =
            new PPMecanumControllerCommand(
                    AutonomousTrajectories.kHolonomicTestPath[0],
                    m_drivetrain::getPose,
                    m_drivetrain.getKinematics(),
                    kXPID,
                    kYPID,
                    kThetaPID,
                    kMaxSpeed,
                    m_drivetrain::setSpeeds,
                    m_drivetrain);

    public HolonomicTestPath() {
        addCommands(
                new InstantCommand(
                        () -> {
                            m_drivetrain.setHeading(getInitialPose().getRotation());
                            m_drivetrain.resetOdometry(getInitialPose());
                        }),
                m_mecConCom,
                new InstantCommand(
                        () -> m_drivetrain.setSpeeds(new MecanumDriveWheelSpeeds(0, 0, 0, 0))));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_drivetrain.stopMotor();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void preview() {}

    @Override
    public Pose2d getInitialPose() {
        return AutonomousTrajectories.kHolonomicTestPath[0].getInitialPose();
    }
}
