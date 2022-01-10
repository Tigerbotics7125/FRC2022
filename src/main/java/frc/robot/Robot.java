
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private RobotContainer container;
	private MecanumDrive robotDrive;
	private Joystick joystick;

	private PigeonIMU pigeon;
	double[] gyro = new double[3];
	double[] accel = new double[3];

	@Override
	public void robotInit() {
		container = new RobotContainer();

		PWMTalonSRX frontLeftMotor = new PWMTalonSRX(0);
		PWMTalonSRX frontRightMotor = new PWMTalonSRX(2);
		PWMTalonSRX rearLeftMotor = new PWMTalonSRX(1);
		PWMTalonSRX rearRightMotor = new PWMTalonSRX(3);

		robotDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		joystick = new Joystick(0);

		pigeon = new PigeonIMU(new TalonSRX(4)); // because pigeon is connected to can through the Talons ribbon cable.
		pigeon.setFusedHeadingToCompass(); // compass is more accurate when not near any mag interference.

	}

	@Override
	public void robotPeriodic() {
		pigeon.getRawGyro(gyro);
		pigeon.getAccelerometerAngles(accel);
	}

	@Override
	public void autonomousInit() {
		container.getAutonomousCommand().schedule();
		pigeon.setFusedHeading(0.0);
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
		robotDrive.driveCartesian(joystick.getX(), joystick.getY(), joystick.getZ(), gyro[2]);
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}

	/**
	 * 
	 * LEAVE DISABLED METHODS ALONE
	 * only overriden to stop the default override me method.
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
