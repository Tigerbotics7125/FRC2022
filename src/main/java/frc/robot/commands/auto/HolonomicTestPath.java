/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import static frc.robot.constants.MecanumDrivetrainConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.command.MecanumControllerCommand;
import frc.lib.util.PreviewableCommand;
import frc.robot.DashboardManager;
import frc.robot.DashboardManager.Tab;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.AutonomousTrajectory;
import frc.robot.constants.MecanumDrivetrainConstants;
import frc.robot.subsystems.MecanumDrivetrainSub;

public class HolonomicTestPath extends SequentialCommandGroup implements PreviewableCommand {

    private static final HolonomicTestPath instance = new HolonomicTestPath();

    private MecanumDrivetrainSub m_drivetrain = RobotContainer.kDrivetrain;
    private AutonomousTrajectory kAuto = AutonomousTrajectory.HOLONOMIC_TEST_PATH_2;

    private Command m_mecConCom =
            new MecanumControllerCommand(
                    kAuto.kTrajs[0],
                    m_drivetrain::getPose,
                    m_drivetrain.getKinematics(),
                    kXPID,
                    kYPID,
                    kThetaPID,
                    /*m_drivetrain::getHeading,*/
                    MecanumDrivetrainConstants.kMaxSpeed,
                    m_drivetrain::setSpeeds,
                    m_drivetrain);

    private HolonomicTestPath() {
        DashboardManager.kField.getObject(this.getName()).setTrajectory(kAuto.kTrajs[0]);

        Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot X Vel Setpoint", kXPID::getSetpoint);
        Shuffleboard.getTab(Tab.AUTO.name).addNumber("Robot Y Vel Setpoint", kYPID::getSetpoint);
        Shuffleboard.getTab(Tab.AUTO.name)
                .addNumber("Robot Theta Vel Setpoint", () -> kThetaPID.getSetpoint().velocity);

        addCommands(m_mecConCom);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_drivetrain.stopMotor();
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
