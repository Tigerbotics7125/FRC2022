/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import static frc.robot.constants.MecanumDrivetrainConstants.kMaxWheelSpeedMPS;
import static frc.robot.constants.MecanumDrivetrainConstants.kThetaPID;
import static frc.robot.constants.MecanumDrivetrainConstants.kXPID;
import static frc.robot.constants.MecanumDrivetrainConstants.kYPID;

import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DashboardManager;
import frc.robot.RobotContainer;
import frc.robot.constants.AutonomousTrajectories;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MecanumDrivetrainSub;
import frc.tigerlib.command.AutonomousCommand;

/**
 * A mostly simple autnomous path to take our preloaded ball, and one ball from the field and score
 * them
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class TwoBallAuto extends SequentialCommandGroup implements AutonomousCommand {

    private MecanumDrivetrainSub m_drivetrain = RobotContainer.kDrivetrain;
    private ArmSub m_arm = RobotContainer.kArm;
    private IntakeSub m_intake = RobotContainer.kIntake;

    private Command m_driveCommand =
            new PPMecanumControllerCommand(
                    AutonomousTrajectories.kTwoBallAuto[0],
                    m_drivetrain::getPose,
                    m_drivetrain.getKinematics(),
                    kXPID,
                    kYPID,
                    kThetaPID,
                    kMaxWheelSpeedMPS,
                    m_drivetrain::setSpeeds,
                    m_drivetrain);

    public TwoBallAuto() {
        addCommands(
                // tell the robot where it is
                new InstantCommand(() -> m_drivetrain.resetOdometry(getInitialPose())),
                // drive around and intake.
                new ParallelCommandGroup(m_driveCommand, new RunCommand(() -> m_intake.intake())),
                // score the balls pls.
                m_arm.getRaiseEjectLowerCommand(),
                // shut off the motors.
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> m_drivetrain.setSpeeds(new MecanumDriveWheelSpeeds())),
                        new InstantCommand(() -> m_intake.disable()),
                        new InstantCommand(() -> m_arm.disable())));
    }

    public Pose2d getInitialPose() {
        return new Pose2d(5.97, 4.71, Rotation2d.fromDegrees(90));
    }

    @Override
    public void preview() {
        DashboardManager.kField
                .getObject("Two Ball Auto")
                .setTrajectory(AutonomousTrajectories.kTwoBallAuto[0]);
    }
}
