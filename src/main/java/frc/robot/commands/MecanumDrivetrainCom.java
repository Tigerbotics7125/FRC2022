package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gamepads;
import frc.robot.subsystems.MecanumDrivetrainSub;

/**
 * Drives a Mecanum
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class MecanumDrivetrainCom extends CommandBase {

  MecanumDrivetrainSub m_drivetrain;

  public MecanumDrivetrainCom(MecanumDrivetrainSub drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrain.driveCartesian(
        -Gamepads.getMecaDriveXJoystick().getX(),
        -Gamepads.getMecaDriveYJoystick().getY(),
        Gamepads.getMecaDriveZJoystick().getZ());
  }

  // Keep command always active.
  @Override
  public boolean isFinished() {
    return false;
  }
}
