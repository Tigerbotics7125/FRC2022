package frc.robot.commands.auto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAutoTrajs;
import frc.robot.Robot;
import frc.robot.subsystems.DiffDrivetrain;;

public class BallToHpThenLayup extends SequentialCommandGroup {

	kAutoTrajs kAuto = kAutoTrajs.BALLTOHPTHENLAYUP;

	public BallToHpThenLayup(DiffDrivetrain drive) {

		
		drive.restartOdometry(kAuto.kTrajs[0].getInitialPose());

		for (int i = 0; i < kAuto.kTrajs.length; i++) {
			Robot.kField.getObject("Auto " + i + " : " + kAuto.kTrajs[i].toString()).setTrajectory(kAuto.kTrajs[i]);

			addCommands(new RamseteCommand(
					kAuto.kTrajs[i],
					drive::getPose,
					new RamseteController(2.0, 0.7),
					drive.getFeedForward(),
					drive.getKinematics(),
					drive::getSpeeds,
					drive.getLeftPIDController(),
					drive.getRightPIDController(),
					drive::setOutput,
					drive));
		}

	}

}
