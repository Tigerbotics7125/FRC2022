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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.MecanumDrivetrainConstants;
import frc.tigerlib.Util;
import frc.tigerlib.command.AutonomousCommand;
import java.util.HashMap;
import java.util.Map;

public class DashboardManager {

    private static final Map<String, ShuffleboardTab> kTabs = new HashMap<>();

    public static final Field2d kField = new Field2d();
    public static final SendableChooser<Command> kAutoChooser = new SendableChooser<Command>();

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
    public static void init() {
        for (Tab t : Tab.values()) {
            kTabs.put(t.name, Shuffleboard.getTab(t.name));
        }
        initTabWidgets();
        initCameras();
    }

    /** Sets up cameras */
    public static void initCameras() {}

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
                .add(RobotContainer.kDrivetrain.getDescription(), RobotContainer.kDrivetrain)
                .withWidget(BuiltInWidgets.kMecanumDrive)
                .withSize(3, 3)
                .withPosition(0, 0);
        // #endregion

        // #region TELEOP
        kTabs.get(Tab.TELEOP.name)
                .add("Drive Sub", RobotContainer.kDrivetrain)
                .withSize(1, 1)
                .withPosition(0, 0);
        kTabs.get(Tab.TELEOP.name)
                .add(RobotContainer.kDrivetrain.getDescription(), RobotContainer.kDrivetrain)
                .withWidget(BuiltInWidgets.kMecanumDrive)
                .withSize(3, 3)
                .withPosition(0, 0);
        // #endregion
    }

    public static void update() {
        kField.setRobotPose(RobotContainer.kDrivetrain.getPose());
        if (kAutoChooser.getSelected() instanceof AutonomousCommand) {
            ((AutonomousCommand) kAutoChooser.getSelected()).preview();
        }
        SmartDashboard.putNumber("Heading", RobotContainer.kDrivetrain.getHeading().getDegrees());
        SmartDashboard.putNumber(
                "xController",
                Util.joystickDeadbandSensitivity(
                        -Gamepads.m_driverFlightJs.getX(),
                        MecanumDrivetrainConstants.kDeadband,
                        Util.scaleInput(Gamepads.m_driverFlightJs.getThrottle(), -1, 1, 1, 5)));
        SmartDashboard.putNumber(
                "yController",
                Util.joystickDeadbandSensitivity(
                        -Gamepads.m_driverFlightJs.getY(),
                        MecanumDrivetrainConstants.kDeadband,
                        Util.scaleInput(Gamepads.m_driverFlightJs.getThrottle(), -1, 1, 1, 5)));
        SmartDashboard.putNumber(
                "zController",
                Util.joystickDeadbandSensitivity(
                        -Gamepads.m_driverFlightJs.getZ(),
                        MecanumDrivetrainConstants.kDeadband,
                        Util.scaleInput(Gamepads.m_driverFlightJs.getThrottle(), -1, 1, 1, 5)));
        SmartDashboard.putNumber(
                "Sensitivity",
                Util.scaleInput(Gamepads.m_driverFlightJs.getThrottle(), -1, 1, 1, 5));
        SmartDashboard.putData("AutoChooser", kAutoChooser);
        SmartDashboard.putString("Chosen Auto Command", kAutoChooser.getSelected().getName());
        SmartDashboard.putBoolean("Turning", RobotContainer.kDrivetrain.getTurning());
        SmartDashboard.putBoolean("Field Oriented", RobotContainer.kDrivetrain.getFieldOriented());
    }
}
