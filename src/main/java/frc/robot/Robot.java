package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAutoTrajs;

public class Robot extends TimedRobot {
  public static Field2d m_field;
  public static kAutoTrajs m_selectedAutoTrajs;
  public static RobotContainer m_container;
  private static SendableChooser<kAutoTrajs> m_autoChooser = new SendableChooser<kAutoTrajs>();

  @Override
  public void robotInit() {
    m_container = new RobotContainer();
    m_field = new Field2d();
    SmartDashboard.putData("field", m_field);

    // auto chooser
    // m_autoChooser.setDefaultOption("Default", kAutoTrajs.DEFAULT);
    m_autoChooser.setDefaultOption("ball to hp then layup", kAutoTrajs.BALLTOHPTHENLAYUP);
    SmartDashboard.putData(m_autoChooser);
  }

  @Override
  public void robotPeriodic() {

    // Ensure the controller(s) are always configured / connected
    Gamepads.configure();
  }

  @Override
  public void autonomousInit() {
    // Stops all previously running commands.
    CommandScheduler.getInstance().cancelAll();
    m_selectedAutoTrajs = m_autoChooser.getSelected();
    SequentialCommandGroup autoCommand =
        m_container.getAutonomousCommand(m_autoChooser.getSelected());

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // Stops all previously running commands.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    Gamepads.resetConfig();
  }

  @Override
  public void disabledPeriodic() {}
}
