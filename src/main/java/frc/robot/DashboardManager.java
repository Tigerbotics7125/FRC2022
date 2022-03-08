/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.auto.NothingAuto;

/**
 * Helps manage dashboards in one location
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class DashboardManager {

	public static final SendableChooser<AutonomousCommand> kAutoChooser = new SendableChooser<AutonomousCommand>();

	/** Creates new tabs for each Tab enum value. */
	public static void init() {
		kAutoChooser.addOption(
			"No Auto", new NothingAuto());
	}

	public static void periodicUpdate() {
		SmartDashboard.putNumber("Heading", RobotContainer.kDrivetrain.getHeading().getDegrees());
		SmartDashboard.putNumber(
				"xController",
				Gamepads.getRobotXInputSpeed());
		SmartDashboard.putNumber(
				"yController",
				Gamepads.getRobotYInputSpeed());
		SmartDashboard.putNumber(
				"zController",
				Gamepads.getRobotZInputSpeed());
		SmartDashboard.putNumber(
				"Sensitivity",
				Gamepads.getScaledThrottle());
		SmartDashboard.putData("AutoChooser", kAutoChooser);
		SmartDashboard.putString("Chosen Auto Command", kAutoChooser.getSelected().getName());
		SmartDashboard.putBoolean("Turning", RobotContainer.kDrivetrain.getTurning());
		SmartDashboard.putBoolean("Field Oriented", RobotContainer.kDrivetrain.getFieldOriented());
	}
}
