/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gamepads;
import static frc.robot.constants.MecanumDrivetrainConstants.kIsFieldOriented;
import frc.robot.subsystems.MecanumDrivetrainSub;
import frc.tigerlib.Util;

/**
 * Drives a Mecanum Drivetrain
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class DriveWithTurning extends CommandBase {

    MecanumDrivetrainSub m_drivetrain;

    public DriveWithTurning(MecanumDrivetrainSub drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = Gamepads.getRobotXSpeed();
        double ySpeed = Gamepads.getRobotYSpeed();
        double zSpeed = Gamepads.getRobotZSpeed();

        m_drivetrain.driveCartesian(ySpeed, xSpeed, zSpeed, kIsFieldOriented ? m_drivetrain.getHeading().getDegrees() : 0.0);
    }

    // Keep command always active.
    @Override
    public boolean isFinished() {
        return false;
    }
}
