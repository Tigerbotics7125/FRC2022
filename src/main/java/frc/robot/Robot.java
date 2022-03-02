/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.DashboardManager.Tab;
import frc.robot.commands.auto.HolonomicTestPath;
import frc.robot.commands.auto.TwoBallAuto;

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
        DashboardManager.kAutoChooser.setDefaultOption(
                "No Auto", new InstantCommand(() -> RobotContainer.kDrivetrain.stopMotor()));
        DashboardManager.kAutoChooser.addOption("HoloTest", new HolonomicTestPath());
        DashboardManager.kAutoChooser.addOption("Two Ball", new TwoBallAuto());

        DashboardManager.init();
        DashboardManager.showTab(Tab.PRE_GAME);
    }

    @Override
    public void robotPeriodic() {
        // Ensure the controller(s) are always configured / connected
        Gamepads.configure();
        // Update the dashboard
        DashboardManager.update();
    }

    @Override
    public void autonomousInit() {
        DashboardManager.showTab(Tab.AUTO);
        // Stops all previously running commands.
        CommandScheduler.getInstance().cancelAll();

        Command autoCommand = DashboardManager.kAutoChooser.getSelected();
        if (autoCommand != null) {
            /*
             * RobotContainer.kDrivetrain.resetOdometry(
             * DashboardManager.kAutoChooser
             * .getSelected()
             * .getInitialPose()); // sets odometry to initial pose.
             */
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
        DashboardManager.showTab(Tab.PRE_GAME);
        CommandScheduler.getInstance().cancelAll();
        Gamepads.resetConfig();
    }

    @Override
    public void disabledPeriodic() {
        DashboardManager.showTab(Tab.PRE_GAME);
    }
}
