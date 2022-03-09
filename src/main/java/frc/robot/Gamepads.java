/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import static frc.robot.constants.MecanumDrivetrainConstants.kDeadband;
import static frc.robot.constants.MecanumDrivetrainConstants.kSensitivity;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.tigerlib.GExtreme3DProJoystick;
import frc.tigerlib.Util;

/**
 * Manages gamepads to control the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Gamepads {
    public static GExtreme3DProJoystick m_driver = new GExtreme3DProJoystick(0);
    public static boolean m_driverConfig = false;

    public static void configure() {
        configureDriver();
    }

    public static void resetConfig() {
        CommandScheduler.getInstance().clearButtons();
        m_driverConfig = false;
        configure();

        if (!m_driverConfig) {
            DriverStation.reportWarning("##### Driver Joystick Not Connected #####", false);
        }
    }

    public static void configureDriver() {
        // Detect whether the joystick has been plugged in after start-up
        if (!m_driverConfig) {
            if (!m_driver.isConnected()) {
                return; // goes back and warns.
            }

            configureBindings();
            m_driverConfig = true;
        }
    }

    public static void configureBindings() {
        // trigger -> eject
        m_driver.kTrigger
                .whileHeld(RobotContainer.kIntake.kEject, true)
                .whenReleased(RobotContainer.kIntake.kDisable);

        // thumb -> intake
        m_driver.kThumb
                .whileHeld(RobotContainer.kIntake.kIntake, true)
                .whenReleased(RobotContainer.kIntake.kDisable);

        // thumb -> intake
        m_driver.kThumb
                .whileHeld(RobotContainer.kArm.kUp, true)
                .whenReleased(RobotContainer.kIntake.kDisable);

        // top3 -> lower arm
        m_driver.kTop3.whenPressed(RobotContainer.kArm.kDown, true);

        // top5 -> raise arm
        m_driver.kTop5.whenPressed(RobotContainer.kArm.kUp, true);
    }

    public static double getRobotXInputSpeed() {
        return Util.joystickDeadbandSensitivity(m_driver.getX(), kDeadband, kSensitivity);
    }

    public static double getRobotYInputSpeed() {
        return Util.joystickDeadbandSensitivity(m_driver.getY(), kDeadband, kSensitivity);
    }

    public static double getRobotZInputSpeed() {
        return Util.joystickDeadbandSensitivity(m_driver.getZ(), kDeadband, kSensitivity);
    }

    public static double getScaledThrottle() {
        return Util.scaleInput(m_driver.getThrottle(), -1, 1, 1, 5);
    }
}
