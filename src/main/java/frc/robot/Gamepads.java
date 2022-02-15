package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.input.ThumbStick;
import frc.lib.input.XboxGamepad;

/**
 * Manages gamepads to control the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Gamepads {
  public static XboxGamepad m_driverXbox = new XboxGamepad(0, 0.15, 0.15);
  public static boolean m_driverXboxConfig = false;
  public static Joystick m_driverFlightJs = new Joystick(1);
  public static boolean m_driverFlightJsConfig = false;

  public static void configure() {
    configureDriverXbox();
    configureDriverFlight();
    // optional other controllers, ie another "driver" who does different things on
    // seperate controller

  }

  public static void resetConfig() {
    CommandScheduler.getInstance().clearButtons();
    m_driverXboxConfig = false;
    m_driverFlightJsConfig = false;
    configure();

    if (!m_driverXboxConfig) {
      DriverStation.reportWarning("##### Driver Xbox Controller Not Connected #####", false);
    }
    if (!m_driverFlightJsConfig) {
      DriverStation.reportWarning("##### Driver Flight Joystick Not Connected #####", false);
    }
  }

  public static void configureDriverXbox() {
    // Detect whether the xbox controller has been plugged in after start-up
    if (!m_driverXboxConfig) {
      if (!m_driverXbox.isConnected()) {
        return; // goes back and should warn.
      }

      driverXboxBindings();
      m_driverXboxConfig = true;
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

  public static void driverXboxBindings() {
    // Driver button controls
  }

  public static void driverFlightBindings() {
    // Driver button Controls
  }

  public static ThumbStick getDiffDriveYJoystick() {
    return m_driverXbox.leftStick;
  }

  public static ThumbStick getDiffDriveXJoystick() {
    return m_driverXbox.leftStick;
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
