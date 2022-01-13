package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.DiffDrivetrain;

public class BallToHpThenLayup extends SequentialCommandGroup {

	private static Path bumpBallToHPPath = Filesystem.getDeployDirectory().toPath()
			.resolve("paths/BallToHP.wpilib.json");
	private static Path terminalToHubPath = Filesystem.getDeployDirectory().toPath()
			.resolve("paths/terminalToHub.wpilib.json");

	public BallToHpThenLayup(DiffDrivetrain drive) {
		addRequirements(drive);

		Trajectory bumpBallToHPTraj;
		try {
			bumpBallToHPTraj = TrajectoryUtil.fromPathweaverJson(bumpBallToHPPath);
			Robot.kField.getObject("ballToHpTraj").setTrajectory(bumpBallToHPTraj);
		} catch (IOException e) {
			DriverStation.reportError("Unable to load Trajectory: " + bumpBallToHPPath, e.getStackTrace());
			return;
		}

		Trajectory terminalToHubTraj;
		try {
			terminalToHubTraj = TrajectoryUtil.fromPathweaverJson(terminalToHubPath);
			Robot.kField.getObject("terminalToHubTraj").setTrajectory(terminalToHubTraj);
		} catch (IOException e) {
			DriverStation.reportError("Unable to load Trajectory: " + terminalToHubPath, e.getStackTrace());
			return;
		}

		addCommands(
				new RamseteCommand(bumpBallToHPTraj, drive::getPose,
						new RamseteController(2.0, 0.7),
						drive.getFeedForward(), drive.getKinematics(), drive::getSpeeds,
						drive.getLeftPIDController(), drive.getRightPIDController(),
						drive::setOutput, drive),

				new RamseteCommand(terminalToHubTraj, drive::getPose,
						new RamseteController(2.0, 0.7),
						drive.getFeedForward(), drive.getKinematics(), drive::getSpeeds,
						drive.getLeftPIDController(), drive.getRightPIDController(),
						drive::setOutput, drive));
	}

}
