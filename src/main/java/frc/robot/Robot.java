
package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAutoTrajs;


public class Robot extends TimedRobot {
	public static Field2d kField;
	public static kAutoTrajs kSelectedAutoTrajs;
	private RobotContainer m_robotContainer;
	private SendableChooser<kAutoTrajs> m_autoChooser = new SendableChooser<kAutoTrajs>();


	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		kField = new Field2d();
		SmartDashboard.putData("field", kField);

		// auto chooser
		//m_autoChooser.setDefaultOption("Default", kAutoTrajs.DEFAULT);
		m_autoChooser.setDefaultOption("ball to hp then layup", kAutoTrajs.BALLTOHPTHENLAYUP);
		SmartDashboard.putData(m_autoChooser);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}
	
	@Override
	public void autonomousInit() {
		// Stops all previously running commands.
		CommandScheduler.getInstance().cancelAll();
		kSelectedAutoTrajs = m_autoChooser.getSelected();
		SequentialCommandGroup autoCommand = m_robotContainer.getAutonomousCommand(m_autoChooser.getSelected());

		if (autoCommand != null) {
			autoCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// Stops all previously running commands.
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void simulationInit() {
		REVPhysicsSim.getInstance().addSparkMax(m_robotContainer.m_drivetrain.m_left, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(m_robotContainer.m_drivetrain.m_right, DCMotor.getNEO(1));
	}

	@Override
	public void simulationPeriodic() {
		REVPhysicsSim.getInstance().run();
	}

	/**
	 * 
	 * LEAVE DISABLED METHODS ALONE
	 * only overridden to stop the default override me message.
	 * 
	 */
	@Override
	public void disabledInit() {
		// DONT PUT STUFF HERE
	}

	@Override
	public void disabledPeriodic() {
		// DONT PUT STUFF HERE
	}

}
