/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

    }

    public static Joystick getMecaDriveYJoystick() {
        return m_driverFlightJs;
    }

    public static Joystick getMecaDriveXJoystick() {
        return m_driverFlightJs;
    }

    public static Joystick getMecaDriveZJoystick() {
        return m_driverFlightJs;
    }
}
