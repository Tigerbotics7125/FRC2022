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
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MecanumDrivetrainSub;

/**
 * Contains all subsystems of the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class RobotContainer {

    public static final PowerDistribution kPdp = new PowerDistribution(0, ModuleType.kCTRE);
    public static final MecanumDrivetrainSub kDrivetrain = new MecanumDrivetrainSub();
    // public static final ArmSub kArm = new ArmSub();
    // public static final IntakeSub kIntake = new IntakeSub();

    public SequentialCommandGroup getAutonomousCommand() {
        switch ((AutonomousTrajectory) DashboardManager.kAutoChooser.getSelected()) {
            case HOLONOMIC_TEST_PATH:
                return HolonomicTestPath.getInstance();
            default:
                // no command, return null to not schedule a command.
                return null;
        }
    }
}
