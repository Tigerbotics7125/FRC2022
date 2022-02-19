/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
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

    private static final Map<String, ShuffleboardTab> kTabs = new HashMap<>();

    private static final Field2d kField = new Field2d();
    private static final SendableChooser<AutonomousTrajectory> kAutoChooser =
            new SendableChooser<AutonomousTrajectory>();

    private static final String kFieldWidget =
            "Field"; // because BuiltInWidgets doesnt contain it yet.

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

    /** Selects the tab shown on the dashboard. */
    public static void showTab(Tab t) {
        Shuffleboard.selectTab(t.name);
    }

    /** Creates new tabs for each Tab enum value. */
    public static void initTabs() {
        for (Tab t : Tab.values()) {
            kTabs.put(t.name, Shuffleboard.getTab(t.name));
        }
        initTabWidgets();
    }

    /** Adds widgets to each Tab */
    private static void initTabWidgets() {
        // #region PRE-GAME
        kTabs.get(Tab.PRE_GAME.name)
                .add("Autonomous Selection", kAutoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(3, 1)
                .withPosition(0, 0);
        kTabs.get(Tab.PRE_GAME.name)
                .add("Field", kField)
                .withWidget(kFieldWidget)
                .withSize(5, 3)
                .withPosition(0, 2);
        // #endregion

        // #region AUTO
        kTabs.get(Tab.AUTO.name)
                .add("Field", kField)
                .withWidget(kFieldWidget)
                .withSize(5, 3)
                .withPosition(0, 2);
        kTabs.get(Tab.AUTO.name)
                .add(RobotContainer.m_drivetrain.getDescription(), RobotContainer.m_drivetrain)
                .withWidget(BuiltInWidgets.kMecanumDrive)
                .withSize(3, 3)
                .withPosition(0, 0);
        // #endregion

        // #region TELEOP
        kTabs.get(Tab.TELEOP.name)
                .add("Drive Sub", RobotContainer.m_drivetrain)
                .withSize(1, 1)
                .withPosition(0, 0);
        kTabs.get(Tab.TELEOP.name)
                .add(RobotContainer.m_drivetrain.getDescription(), RobotContainer.m_drivetrain)
                .withWidget(BuiltInWidgets.kMecanumDrive)
                .withSize(3, 3)
                .withPosition(0, 0);
        // #endregion
    }

    public static Field2d getField() {
        return kField;
    }

    public static SendableChooser<AutonomousTrajectory> getAutoChooser() {
        return kAutoChooser;
    }
}
