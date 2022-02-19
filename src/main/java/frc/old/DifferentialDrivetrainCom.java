/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.old;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drives a DifferentialDrivetrain
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 * @deprecated
 */
@Deprecated
public class DifferentialDrivetrainCom extends CommandBase {

    DifferentialDrivetrainSub m_drivetrain;

    public DifferentialDrivetrainCom(DifferentialDrivetrainSub drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        /*
        WheelSpeeds ws =
            DifferentialDrive.arcadeDriveIK(
                Gamepads.getDiffDriveYJoystick().getY(), Gamepads.getDiffDriveXJoystick().getX(), true);
        m_drivetrain.setOutput(ws.left * 12.0, ws.right * 12.0);
        */
    }

    // Keep command always active.
    @Override
    public boolean isFinished() {
        return false;
    }
}
