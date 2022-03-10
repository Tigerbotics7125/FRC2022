/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
        m_AutoChooser.addOption("No Auto", new NothingAuto());

        RobotContainer.kCamera1.setFPS(30);
        RobotContainer.kCamera2.setFPS(30);
        RobotContainer.kCamera1.setResolution(320, 240);
        RobotContainer.kCamera2.setResolution(320, 240);
        RobotContainer.kCamera1.setExposureManual(50);
        RobotContainer.kCamera2.setExposureManual(50);
        RobotContainer.kCamera1.setBrightness(50);
        RobotContainer.kCamera2.setBrightness(50);
        RobotContainer.kCamera1.setWhiteBalanceManual(50);
        RobotContainer.kCamera2.setWhiteBalanceManual(50);
        RobotContainer.kCamera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        RobotContainer.kCamera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    }

    public void updateValues() {
        // Controller
        SmartDashboard.putNumber("x", Gamepads.getRobotXInputSpeed());
        SmartDashboard.putNumber("yInpt", Gamepads.getRobotYInputSpeed());
        SmartDashboard.putNumber("zInput", Gamepads.getRobotZInputSpeed());
        SmartDashboard.putNumber("tInput", Gamepads.getScaledThrottle());

        // Auto
        SmartDashboard.putData("AutoChooser", m_AutoChooser);
        SmartDashboard.putString("Chosen Auto:", m_AutoChooser.getSelected().getName());

        // Robot Info
        SmartDashboard.putNumber("Heading", RobotContainer.kDrivetrain.getHeading().getDegrees());
        SmartDashboard.putBoolean("Arm Up", RobotContainer.kArm.getFwdLimitSwitch());
        SmartDashboard.putBoolean("Arm Down", RobotContainer.kArm.getRevLimitSwitch());

        // Driving Options
        SmartDashboard.putBoolean("Turning", RobotContainer.kDrivetrain.getTurning());
        SmartDashboard.putBoolean("Field Orient", RobotContainer.kDrivetrain.getFieldOriented());

        // Subsystems
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("PDP", RobotContainer.kPdp);
        SmartDashboard.putData("Drivetrain", RobotContainer.kDrivetrain);
        SmartDashboard.putData("Arm", RobotContainer.kArm);
        SmartDashboard.putData("Intake", RobotContainer.kIntake);

        // Puts all sendable data to the dashboard.
        SmartDashboard.updateValues();
    }

    /** Gets the currently chosen autonomous command. */
    public AutonomousCommand getChosenAutonomousCommand() {
        return m_AutoChooser.getSelected();
    }
}
