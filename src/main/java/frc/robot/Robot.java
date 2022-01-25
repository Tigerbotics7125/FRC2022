package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAutoTrajs;

/**
 * The main robot class, runs all loops and main control
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Robot extends TimedRobot {
  public static Field2d m_field;
  public static kAutoTrajs m_selectedAutoTrajs;
  public static RobotContainer m_container;

  // Options from dashboard.
  private static SendableChooser<kAutoTrajs> m_autoChooser = new SendableChooser<kAutoTrajs>();

  // Shuffleboard ... pain...
  public static ShuffleboardTab m_preTab;
  public static ShuffleboardTab m_autoTab;
  public static ShuffleboardTab m_teleopTab;

  @Override
  public void robotInit() {
    // initialize variables
    m_container = new RobotContainer();
    m_field = new Field2d();

    // auto chooser
    // temporary default until we have an actual default.
    // m_autoChooser.setDefaultOption("Default", kAutoTrajs.DEFAULT);
    m_autoChooser.setDefaultOption("ball to hp then layup", kAutoTrajs.BALLTOHPTHENLAYUP);

    // initialize shuffleboard tabs
    initPreGameTab();
    initAutoTab();
    initTeleopTab();
    Shuffleboard.selectTab(kPreGameTabName);
  }

  private void initPreGameTab() {
    m_preTab = Shuffleboard.getTab(kPreGameTabName);
    m_preTab
        .add("Auto Select", m_autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2, 1)
        .withPosition(0, 0);
    m_preTab.add("Field", m_field).withWidget("Field").withSize(5, 3).withPosition(0, 2);
  }

  private void initAutoTab() {
    m_autoTab = Shuffleboard.getTab(kAutoTabName);
    m_autoTab.add("Field", m_field).withWidget("Field").withSize(5, 3).withPosition(0, 2);
    // m_autoTab.add("Drivetrain",
    // m_container.m_drivetrain.drive).withWidget(BuiltInWidgets.kDifferentialDrive).withPosition(5,
    // 0);
  }

  private void initTeleopTab() {
    m_teleopTab = Shuffleboard.getTab(kTeleopTabName);
  }

  @Override
  public void robotPeriodic() {
    // Ensure the controller(s) are always configured / connected
    Gamepads.configure();

    // preview the selected auto trajectory
    if (RobotState.isDisabled() && m_field != null && m_autoChooser != null) {
      m_field.getObject("preview").setTrajectory(m_autoChooser.getSelected().kFullTraj);
      m_field.getObject("preview").setPose(m_autoChooser.getSelected().kFullTraj.getInitialPose());
    }
  }

  @Override
  public void autonomousInit() {
    Shuffleboard.selectTab(kAutoTabName);
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
    Shuffleboard.selectTab(kTeleopTabName);
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
