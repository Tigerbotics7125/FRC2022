package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.PreviewableCommand;
import frc.robot.DashboardManager;
import frc.robot.DashboardManager.Tab;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.AutonomousTrajectory;
import frc.robot.constants.MecanumDrivetrainConstants;
import frc.robot.subsystems.MecanumDrivetrainSub;

public class HolonomicTestPath extends SequentialCommandGroup implements PreviewableCommand {

  private static final HolonomicTestPath instance = new HolonomicTestPath();

  private MecanumDrivetrainSub m_drivetrain = RobotContainer.m_drivetrain;
  private AutonomousTrajectory kAuto = AutonomousTrajectory.HOLONOMIC_TEST_PATH;

  private PIDController m_xPID = new PIDController(1, 0, 0);
  private PIDController m_yPID = new PIDController(1, 0, 0);
  private ProfiledPIDController m_thetaPID =
      new ProfiledPIDController(1, 0, 0, new Constraints(6.28, 3.14));

  private Command m_mecConCom =
      new MecanumControllerCommand(
          kAuto.kTrajs[0],
          m_drivetrain::getPose,
          m_drivetrain.getKinematics(),
          m_xPID,
          m_yPID,
          m_thetaPID,
          m_drivetrain::getHeading,
          MecanumDrivetrainConstants.kMaxSpeed,
          m_drivetrain::setSpeeds,
          m_drivetrain);

  private HolonomicTestPath() {
    DashboardManager.getField().getObject(this.getName()).setTrajectory(kAuto.kTrajs[0]);

    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot Target X Vel", m_xPID::getSetpoint);
    Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot Target Y Vel", m_yPID::getSetpoint);
    Shuffleboard.getTab(Tab.AUTO.name)
        .addNumber("Robot Target Rot Deg", () -> m_thetaPID.getSetpoint().position);

    addCommands(m_mecConCom);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void preview() {
    // TODO Auto-generated method stub

  }

  public static HolonomicTestPath getInstance() {
    return instance;
  }
}
