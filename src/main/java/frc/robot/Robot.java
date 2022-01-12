
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	public RobotState m_state;
	private RobotContainer m_robotContainer;
	private Command m_autonomousCommand;

	public static enum RobotState {
		DISABLED, AUTO, TELEOP, TEST
	}

	@Override
	public void robotInit() {
		m_state = RobotState.DISABLED;
		m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		m_state = RobotState.AUTO;
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		m_state = RobotState.TELEOP;
		// Stops autonomous driving once teleop starts.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void simulationInit() {
		m_state = RobotState.TEST;
	}

	@Override
	public void simulationPeriodic() {
	}

	/**
	 * 
	 * LEAVE DISABLED METHODS ALONE
	 * only overridden to stop the default override me message.
	 * 
	 */
	@Override
	public void disabledInit() {
		m_state = RobotState.DISABLED;
		// DONT PUT STUFF HERE
	}

	@Override
	public void disabledPeriodic() {
		// DONT PUT STUFF HERE
	}

}
