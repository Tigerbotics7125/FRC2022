package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.Constants.AutonomousTrajectory;
import java.util.HashMap;
import java.util.Map;

public class DashboardManager {

  private static Map<String, ShuffleboardTab> m_tabs = new HashMap<>();

  private static Field2d m_field = new Field2d();
  private static SendableChooser<AutonomousTrajectory> m_autoChooser =
      new SendableChooser<AutonomousTrajectory>();

  private static final String kField = "Field"; // because BuiltInWidgets doesnt contain it yet.

  /** Defined tabs to keep things consistant */
  public static enum Tab {
    PRE_GAME("Pre-Game"),
    AUTO("Auto"),
    TELEOP("Teleop");

    public final String name;

    Tab(String name) {
      this.name = name;
    }
  }

  public static SendableChooser<AutonomousTrajectory> getAutoChooser() {
    return m_autoChooser;
  }

  /** Selects the tab shown on the dashboard. */
  public static void showTab(Tab t) {
    Shuffleboard.selectTab(t.name);
  }

  /** Creates new tabs for each Tab enum value. */
  public static void initTabs() {
    for (Tab t : Tab.values()) {
      m_tabs.put(t.name, Shuffleboard.getTab(t.name));
    }
    initTabWidgets();
  }

  /** Adds widgets to each Tab */
  private static void initTabWidgets() {
    // PRE-GAME
    m_tabs
        .get(Tab.PRE_GAME.name)
        .add("Autonomous Selection", m_autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(3, 1)
        .withPosition(0, 0);
    m_tabs
        .get(Tab.PRE_GAME.name)
        .add("Field", m_field)
        .withWidget(kField)
        .withSize(5, 3)
        .withPosition(0, 2);

    // AUTO
    m_tabs
        .get(Tab.AUTO.name)
        .add("Field", m_field)
        .withWidget(kField)
        .withSize(5, 3)
        .withPosition(0, 2);

    // TELEOP
    m_tabs
        .get(Tab.TELEOP.name)
        .add("Drive Sub", RobotContainer.m_drivetrain)
        .withSize(1, 1)
        .withPosition(0, 0);
  }

  public static Field2d getField() {
    return m_field;
  }
}
