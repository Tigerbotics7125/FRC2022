/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gamepads;
import frc.robot.subsystems.MecanumDrivetrainSub;

/**
 * Drives a Mecanum Drivetrain
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class MecanumDrivetrainCom extends CommandBase {

    MecanumDrivetrainSub m_drivetrain;

    public MecanumDrivetrainCom(MecanumDrivetrainSub drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drivetrain.driveCartesian(
                -Gamepads.m_driverFlightJs.getX(),
                -Gamepads.m_driverFlightJs.getY(),
                Gamepads.m_driverFlightJs.getZ());
    }

    // Keep command always active.
    @Override
    public boolean isFinished() {
        return false;
    }
}
