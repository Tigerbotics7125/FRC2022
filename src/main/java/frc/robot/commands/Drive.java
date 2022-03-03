/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDrivetrainSub;

/**
 * Runs the drivetrain as a command; actuall driving is done in drivetrain class.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Drive extends CommandBase {

    MecanumDrivetrainSub m_drivetrain;

    public Drive(MecanumDrivetrainSub drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drivetrain.drive(); // all is handled in subsystem, just need to call it.
    }

    // Keep command always active.
    @Override
    public boolean isFinished() {
        return false;
    }
}
