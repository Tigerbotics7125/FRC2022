/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DashboardManager.Tab;
import frc.robot.constants.Constants.AutonomousTrajectory;

/**
 * The main robot class, runs all loops and main control
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Robot extends TimedRobot {
    public static RobotContainer m_container;

    @Override
    public void robotInit() {
        // initialize variables
        m_container = new RobotContainer();

        // auto chooser
        // temporary default until we have an actual default.
        // m_autoChooser.setDefaultOption("Default", kAutoTrajs.DEFAULT);
        DashboardManager.getAutoChooser()
                .setDefaultOption("ball to hp then layup", AutonomousTrajectory.BALLTOHPTHENLAYUP);
        DashboardManager.getAutoChooser()
                .addOption("HoloTest", AutonomousTrajectory.HOLONOMIC_TEST_PATH);
        DashboardManager.initTabs();
        DashboardManager.showTab(Tab.PRE_GAME);
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
        SequentialCommandGroup autoCommand =
                m_container.getAutonomousCommand(DashboardManager.getAutoChooser().getSelected());
        if (autoCommand != null) {
            RobotContainer.m_drivetrain.resetOdometry(
                    DashboardManager.getAutoChooser()
                            .getSelected()
                            .kTrajs[0]
                            .getInitialPose()); // sets odometry to initial pose.
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        DashboardManager.showTab(Tab.TELEOP);
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
