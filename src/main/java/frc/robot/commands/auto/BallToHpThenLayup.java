package frc.robot.commands.auto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAutoTrajs;
import frc.robot.Robot;
import frc.robot.subsystems.DifferentialDrivetrain;

public class BallToHpThenLayup extends SequentialCommandGroup {

  DifferentialDrivetrain m_drivetrain;
  kAutoTrajs kAuto = kAutoTrajs.BALLTOHPTHENLAYUP;

  public BallToHpThenLayup(DifferentialDrivetrain drivetrain) {

    m_drivetrain = drivetrain;
    //addRequirements(m_drivetrain);

    for (int i = 0; i < kAuto.kTrajs.length; i++) {
      Robot.m_field
          .getObject("Auto " + i)
          .setTrajectory(kAuto.kTrajs[i]);

      addCommands(
          new RamseteCommand(
              kAuto.kTrajs[i],
              m_drivetrain::getPose,
              new RamseteController(2.0, 0.7),
              m_drivetrain.getFeedForward(),
              m_drivetrain.getKinematics(),
              m_drivetrain::getSpeeds,
              m_drivetrain.getLeftPIDController(),
              m_drivetrain.getRightPIDController(),
              m_drivetrain::setOutput,
              m_drivetrain));
    }
  }

  @Override
  public void initialize() {
    super.initialize(); // super important about 2 hours worth of debugging unfortunatly
    
    m_drivetrain.restartOdometry(kAuto.kTrajs[0].getInitialPose());
  }
}
