/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main robot class, runs all loops and main control
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Robot extends TimedRobot {

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        // Ensure the controller(s) are always configured / connected
        Gamepads.configure();
        // Update the dashboard
        DashboardManager.getInstance().updateValues();
        // Run any scheduled commands
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        // Stops all previously running commands.
        CommandScheduler.getInstance().cancelAll();

        Command autoCommand = DashboardManager.getInstance().getChosenAutonomousCommand();
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // weep a smither.
    }

    @Override
    public void teleopInit() {
        // Stops all previously running commands.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {}

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
