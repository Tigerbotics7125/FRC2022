package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MecanumDrivetrainSub;
import frc.tigerlib.command.AutonomousCommand;

public class ExitTarmac extends SequentialCommandGroup implements AutonomousCommand {

    private MecanumDrivetrainSub m_drivetrain = RobotContainer.kDrivetrain;

    public ExitTarmac() {
        /*
         * addCommands(
         * new ParallelRaceGroup(
         * new RunCommand(() -> m_drivetrain.drive(0, 1, 0)).withTimeout(.5),
         * new WaitCommand(1)));
         */
        addCommands(new InstantCommand(() -> m_drivetrain.drive(0, -1, 0)).withTimeout(.5),
        new RunCommand(() -> RobotContainer.kIntake.eject()).withTimeout(1),
        new InstantCommand(() -> RobotContainer.kIntake.disable()));
    }

    @Override
    public Pose2d getInitialPose() {
        // TODO Auto-generated method stub
        return new Pose2d();
    }

    @Override
    public void preview() {
        // TODO Auto-generated method stub

    }

}