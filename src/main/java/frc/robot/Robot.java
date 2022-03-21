/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main robot class, runs all loops and main control
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Robot extends TimedRobot {

    RobotContainer mContainer;

    @Override
    public void robotInit() {
        mContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Update the dashboard
        mContainer.updateValues();
        // Run any scheduled commands
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        Command autoCommand = mContainer.getSelectedAuto();
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // Stops all previously running commands.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {}
}
