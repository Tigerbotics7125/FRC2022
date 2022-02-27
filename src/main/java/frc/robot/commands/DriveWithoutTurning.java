package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gamepads;
import static frc.robot.constants.MecanumDrivetrainConstants.kIsFieldOriented;
import frc.robot.subsystems.MecanumDrivetrainSub;

public class DriveWithoutTurning extends CommandBase{
    
    MecanumDrivetrainSub m_drivetrain;
    
    public DriveWithoutTurning(MecanumDrivetrainSub drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double ySpeed = Gamepads.getRobotYSpeed();
        double xSpeed = Gamepads.getRobotXSpeed();

        m_drivetrain.driveCartesian(ySpeed, xSpeed, 0.0, kIsFieldOriented ? m_drivetrain.getHeading().getDegrees() : 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
