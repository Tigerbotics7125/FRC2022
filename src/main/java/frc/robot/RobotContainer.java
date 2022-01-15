package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAutoTrajs;
import frc.robot.commands.auto.BallToHpThenLayup;
import frc.robot.subsystems.DifferentialDrivetrain;

public class RobotContainer {

  public DifferentialDrivetrain m_drivetrain;

  public RobotContainer() {
    m_drivetrain = new DifferentialDrivetrain();
  }

  public SequentialCommandGroup getAutonomousCommand(kAutoTrajs auto) {
    switch (auto) {
      case BALLTOHPTHENLAYUP:
        return new BallToHpThenLayup(m_drivetrain);
      default:
        return null;
    }
  }
}
