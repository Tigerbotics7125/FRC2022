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
import frc.robot.constants.AutonomousTrajectories;

/**
 * A simple test path to test autonomous driving our mecanum drivetrain.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class HolonomicTestPath extends AutonomousCommand {

    private Command m_mecConCom =
            new PPMecanumControllerCommand(
                    AutonomousTrajectories.kHolonomicTestPath[0],
                    kDrivetrain::getPose,
                    kDrivetrain.getKinematics(),
                    kXPID,
                    kYPID,
                    kThetaPID,
                    kMaxSpeed,
                    kDrivetrain::setSpeeds,
                    kDrivetrain);

    public HolonomicTestPath() {
        addCommands(
                new InstantCommand(
                        () -> {
                            kDrivetrain.setHeading(getInitialPose().getRotation());
                            kDrivetrain.resetOdometry(getInitialPose());
                        }),
                m_mecConCom,
                new InstantCommand(
                        () -> kDrivetrain.setSpeeds(new MecanumDriveWheelSpeeds(0, 0, 0, 0))));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
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

    @Override
    public String getName() {
        return "HoloTestPath";
    }
}
