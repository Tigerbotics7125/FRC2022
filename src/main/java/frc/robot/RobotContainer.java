/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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

    public static PowerDistribution m_pdp = new PowerDistribution(0, ModuleType.kCTRE);
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
