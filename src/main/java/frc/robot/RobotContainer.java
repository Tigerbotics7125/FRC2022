package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAutoTrajs;
import frc.robot.commands.auto.BallToHpThenLayup;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.MecanumDrivetrain;

/**
 * Contains all subsystems of the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class RobotContainer {

  public PowerDistribution m_pdp; // CAN id must be 0 for CTRE pdp, and 1 for REV pdh.
  public DifferentialDrivetrain m_drivetrain;
  public MecanumDrivetrain m_mecanumDrivetrain;

  public RobotContainer() {
    m_pdp = new PowerDistribution();
    m_drivetrain = new DifferentialDrivetrain();
    m_mecanumDrivetrain = new MecanumDrivetrain();
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
