package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.BallToHpThenLayup;
import frc.robot.subsystems.DiffDrivetrain;

public class RobotContainer {

	DiffDrivetrain m_drivetrain;

	public RobotContainer() {
		m_drivetrain = new DiffDrivetrain();
	}

	public Command getAutonomousCommand() {
		return new BallToHpThenLayup(m_drivetrain);
	}

	// This is for drivetrain, but there is a holonomicDrive stuff, which allows us
	// to use our mecanum stuff during auton

	/*
	 * 
	 * private Drivetrain drive1 = new Drivetrain();
	 * public Command getStandardDrivetrainAuton() {
	 * TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2),
	 * Units.feetToMeters(2));
	 * config.setKinematics(mDrive.getKinematics());
	 * 
	 * // creates a new list of poses, starting pose, then 1 meter forward. config
	 * Trajectory trajectory = TrajectoryGenerator
	 * .generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new
	 * Rotation2d())), config);
	 * 
	 * // tested constants supposed to work.
	 * RamseteCommand command = new RamseteCommand(trajectory, drive1::getPose, new
	 * RamseteController(2.0, 0.7),
	 * drive1.getFeedForward(), drive1.getKinematics(), drive1::getSpeeds,
	 * drive1.getLeftPIDController(), drive1.getRightPIDController(),
	 * drive1::setOutput, drive1);
	 * 
	 * return command;
	 * }
	 * 
	 * 
	 * private MDrivetrain mDrive = new MDrivetrain();
	 * 
	 * public Command getAutonomousCommand() {
	 * 
	 * TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2),
	 * Units.feetToMeters(2));
	 * config.setKinematics(mDrive.getKinematics());
	 * Trajectory trajectory = TrajectoryGenerator
	 * .generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new
	 * Rotation2d())), config);
	 * 
	 * MecanumControllerCommand cmd = new MecanumControllerCommand(trajectory,
	 * mDrive::getPose, mDrive.getFeedforward(),
	 * mDrive.getKinematics(), mDrive.getXController(), mDrive.getYController(),
	 * mDrive.getThetaController(), 5.0,
	 * mDrive.getFlPIDController(), mDrive.getRlPIDController(),
	 * mDrive.getFrPIDController(),
	 * mDrive.getRrPIDController(), mDrive::getWheelSpeeds,
	 * mDrive::setMotorOutput, mDrive);
	 * 
	 * return cmd;
	 * 
	 * }
	 */

}
