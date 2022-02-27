/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gamepads;
import frc.robot.constants.MecanumDrivetrainConstants;
import frc.robot.subsystems.MecanumDrivetrainSub;
import frc.tigerlib.Util;

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
                Util.joystickDeadbandSensitivity(
                        -Gamepads.m_driverFlightJs.getY(),
                        MecanumDrivetrainConstants.kDeadband,
                        Util.scaleInput(Gamepads.m_driverFlightJs.getThrottle(), -1, 1, 1, 5)),
                Util.joystickDeadbandSensitivity(
                        Gamepads.m_driverFlightJs.getX(),
                        MecanumDrivetrainConstants.kDeadband,
                        Util.scaleInput(Gamepads.m_driverFlightJs.getThrottle(), -1, 1, 1, 5)),
                Util.joystickDeadbandSensitivity(
                        Gamepads.m_driverFlightJs.getZ(),
                        MecanumDrivetrainConstants.kDeadband,
                        Util.scaleInput(Gamepads.m_driverFlightJs.getThrottle(), -1, 1, 1, 5)));
    }

    // Keep command always active.
    @Override
    public boolean isFinished() {
        return false;
    }
}
