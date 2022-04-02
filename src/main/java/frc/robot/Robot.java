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

    // Toys'R'Us but for a robot.
    RobotContainer mContainer;

    /** Init, duh. */
    @Override
    public void robotInit() {
        mContainer = new RobotContainer();
    }

    /** A method that runs every 20ms, no matter what. */
    @Override
    public void robotPeriodic() {
        // Update the dashboard
        mContainer.updateValues();
        // Run any scheduled commands
        CommandScheduler.getInstance().run();
    }

    /** A method that runs before running {@link autonomousPeriodic} loop the first time. */
    @Override
    public void autonomousInit() {
        Command autoCommand = mContainer.getSelectedAuto();
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    /**
     * A method that runs every 20ms when {@link edu.wpi.first.wpilibj.RobotState#isAutonomous()}.
     */
    @Override
    public void autonomousPeriodic() {}

    /** A method that runs before running {@link teleopPeriodic} loop the first time. */
    @Override
    public void teleopInit() {
        // Stops all previously running commands.
        CommandScheduler.getInstance().cancelAll();
    }

    /** A method that runs very 20ms when {@link edu.wpi.first.wpilibj.RobotState#isTeleop()}. */
    @Override
    public void teleopPeriodic() {}

    /** A method that runs before running {@link simulationPeriodic} loop the first time. */
    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * A method that runs every 20ms when {@link edu.wpi.first.wpilibj.RobotBase#isSimulation()}.
     */
    @Override
    public void simulationPeriodic() {}

    /** A method that runs before running {@link disabledPeriodic} loop the first time. */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** A method that runs every 20ms when {@link edu.wpi.first.wpilibj.RobotState#isDisabled()}. */
    @Override
    public void disabledPeriodic() {}
}
