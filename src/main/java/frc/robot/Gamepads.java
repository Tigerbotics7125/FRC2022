/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import static frc.robot.constants.MecanumDrivetrainConstants.kDeadband;
import static frc.robot.constants.MecanumDrivetrainConstants.kIsUsingThrottle;
import static frc.robot.constants.MecanumDrivetrainConstants.kSensitivity;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveWithTurning;
import frc.robot.commands.DriveWithoutTurning;
import frc.tigerlib.Util;

/**
 * Manages gamepads to control the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Gamepads {
    public static Joystick m_driverFlightJs = new Joystick(0);
    public static boolean m_driverFlightJsConfig = false;

    public static void configure() {
        configureDriverFlight();
        // optional other controllers, ie another "driver" who does different things on
        // seperate controller

    }

    public static void resetConfig() {
        CommandScheduler.getInstance().clearButtons();
        m_driverFlightJsConfig = false;
        configure();

        if (!m_driverFlightJsConfig) {
            DriverStation.reportWarning("##### Driver Flight Joystick Not Connected #####", false);
        }
    }

    public static void configureDriverFlight() {
        // Detect whether the joystick has been plugged in after start-up
        if (!m_driverFlightJsConfig) {
            if (!m_driverFlightJs.isConnected()) {
                return; // goes back and warns.
            }

            driverFlightBindings();
            m_driverFlightJsConfig = true;
        }
    }

    public static void driverFlightBindings() {
        // Driver button Controls

        // when thumb is held, will only strafe, if released, will turn and strafe.
        new Button(() -> m_driverFlightJs.getRawButton(1 /* thumb button */))
                .whenReleased(new DriveWithoutTurning(RobotContainer.kDrivetrain))
                .whenHeld(new DriveWithTurning(RobotContainer.kDrivetrain));
    }

    public static double getRobotXSpeed() {
        return Util.joystickDeadbandSensitivity(
                m_driverFlightJs.getX(),
                kDeadband,
                kIsUsingThrottle ? Util.scaleInput(m_driverFlightJs.getThrottle(), -1, 1, 1, 5) : kSensitivity);
    }

    public static double getRobotYSpeed() {
        return Util.joystickDeadbandSensitivity(
                -m_driverFlightJs.getY(),
                kDeadband,
                kIsUsingThrottle ? Util.scaleInput(m_driverFlightJs.getThrottle(), -1, 1, 1, 5) : kSensitivity);
    }

    public static double getRobotZSpeed() {
        return Util.joystickDeadbandSensitivity(
                m_driverFlightJs.getZ(),
                kDeadband,
                kIsUsingThrottle ? Util.scaleInput(m_driverFlightJs.getThrottle(), -1, 1, 1, 5) : kSensitivity);
    }

}
