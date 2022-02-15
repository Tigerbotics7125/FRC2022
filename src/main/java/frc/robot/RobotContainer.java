package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.HolonomicTestPath;
import frc.robot.constants.Constants.AutonomousTrajectory;
import frc.robot.subsystems.MecanumDrivetrainSub;

/**
 * Contains all subsystems of the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class RobotContainer {

  public static PowerDistribution m_pdp =
      new PowerDistribution(); // CAN id must be 0 for CTRE pdp, and 1 for REV pdh.
  // public static DifferentialDrivetrainSub m_drivetrain = new DifferentialDrivetrainSub();
  public static MecanumDrivetrainSub m_drivetrain = new MecanumDrivetrainSub();

  public RobotContainer() {}

  public SequentialCommandGroup getAutonomousCommand(AutonomousTrajectory auto) {
    switch (auto) {
      case HOLONOMIC_TEST_PATH:
        return HolonomicTestPath.getInstance();
      default:
        return null;
    }
  }
}
