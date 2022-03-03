/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import static frc.robot.constants.MecanumDrivetrainConstants.kDeadband;
import static frc.robot.constants.MecanumDrivetrainConstants.kSensitivity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
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

        // when trigger is held, field oriented is enabled, disabled otherwise
        /*
        new Button(() -> Gamepads.m_driverFlightJs.getRawButton(1))
                .whenPressed(() -> RobotContainer.kDrivetrain.setFieldOriented(true))
                .whenReleased(() -> RobotContainer.kDrivetrain.setFieldOriented(false));
        */
        // when thumb button is held, turning is enabled, disabled otherwise
        /*
        new Button(() -> Gamepads.m_driverFlightJs.getRawButton(2))
                .whenPressed(() -> RobotContainer.kDrivetrain.setTurning(true))
                .whenReleased(() -> RobotContainer.kDrivetrain.setTurning(false));
        */

        // when thumb is held down, intake.
        new Button(() -> Gamepads.m_driverFlightJs.getRawButton(2)).whileHeld(() -> RobotContainer.kIntake.intake()).whenReleased(() -> RobotContainer.kIntake.disable());

        // when trigger is pressed, raise arm then eject, then lower arm.
        new Button(() -> Gamepads.m_driverFlightJs.getRawButton(1)).whenPressed(RobotContainer.kArm.getRaiseEjectLowerCommand());

        // when button 5 is pressed lift arm
        new Button(() -> m_driverFlightJs.getRawButton(5)).whenPressed(() -> RobotContainer.kArm.setUp());

        // when button 3 is pressed lower arm
        new Button(() -> m_driverFlightJs.getRawButton(3)).whenPressed(() -> RobotContainer.kArm.setDown());

        // when button 11 is pressed reset odometry
        new Button(() -> m_driverFlightJs.getRawButton(11)).whenPressed(() -> RobotContainer.kDrivetrain.setHeading(Rotation2d.fromDegrees(0)));

    }

    public static double getRobotXInputSpeed() {
        return Util.joystickDeadbandSensitivity(
                m_driverFlightJs.getX(),
                kDeadband,
                kSensitivity);
    }

    public static double getRobotYInputSpeed() {
        return Util.joystickDeadbandSensitivity(
                -m_driverFlightJs.getY(),
                kDeadband,
                kSensitivity);
    }

    public static double getRobotZInputSpeed() {
        return Util.joystickDeadbandSensitivity(
                m_driverFlightJs.getZ(),
                kDeadband,
                kSensitivity);
    }
}
