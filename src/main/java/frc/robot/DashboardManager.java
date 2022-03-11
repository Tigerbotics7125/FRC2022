/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
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

    /** Singelton */
    private static DashboardManager kInstance = new DashboardManager();

    public static DashboardManager getInstance() {
        return kInstance;
    }

    private static SendableChooser<AutonomousCommand> m_AutoChooser;

    private DashboardManager() {
        m_AutoChooser = new SendableChooser<AutonomousCommand>();
        m_AutoChooser.setDefaultOption("No Auto", new NothingAuto());

        RobotContainer.kCamera1.setFPS(30);
        RobotContainer.kCamera2.setFPS(30);
        RobotContainer.kCamera1.setWhiteBalanceAuto();
        RobotContainer.kCamera2.setWhiteBalanceAuto();
        RobotContainer.kCamera1.setExposureAuto();
        RobotContainer.kCamera2.setExposureAuto();
        RobotContainer.kCamera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        RobotContainer.kCamera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    }

    public void updateValues() {
        // Controller
        SmartDashboard.putNumber("xInput", Gamepads.getRobotXInputSpeed());
        SmartDashboard.putNumber("yInput", Gamepads.getRobotYInputSpeed());
        SmartDashboard.putNumber("zInput", Gamepads.getRobotZInputSpeed());
        SmartDashboard.putNumber("tInput", Gamepads.getScaledThrottle());

        // Auto
        SmartDashboard.putData("AutoChooser", m_AutoChooser);
        SmartDashboard.putString("Chosen Auto", m_AutoChooser.getSelected().getName());

        // Robot Info
        SmartDashboard.putNumber("Heading", RobotContainer.kDrivetrain.getHeading().getDegrees());
        SmartDashboard.putBoolean("Is up?", RobotContainer.kArm.getFwdLimitSwitch());
        SmartDashboard.putBoolean("Is down?", RobotContainer.kArm.getRevLimitSwitch());

        // Driving Options
        SmartDashboard.putBoolean("Turning?", RobotContainer.kDrivetrain.getTurning());
        SmartDashboard.putBoolean("Field Oriented?", RobotContainer.kDrivetrain.getFieldOriented());

        // Subsystems
        // SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
        // SmartDashboard.putData("PDP", RobotContainer.kPdp);
        String drivetrainCmd = RobotContainer.kDrivetrain.getCurrentCommand().getName();
        SmartDashboard.putString("Drivetrain", drivetrainCmd);
        SmartDashboard.putString("Arm", RobotContainer.kArm.getCurrentCommand().getName());
        SmartDashboard.putString("Intake", RobotContainer.kIntake.getCurrentCommand().getName());

        // Puts all sendable data to the dashboard.
        SmartDashboard.updateValues();
    }

    /** Gets the currently chosen autonomous command. */
    public AutonomousCommand getChosenAutonomousCommand() {
        return m_AutoChooser.getSelected();
    }
}
