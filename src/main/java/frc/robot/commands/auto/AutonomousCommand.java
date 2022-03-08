/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.MecanumDrivetrainSub;

public abstract class AutonomousCommand extends SequentialCommandGroup {

    protected static final MecanumDrivetrainSub kDrivetrain = RobotContainer.kDrivetrain;
    protected static final ArmSub kArm = RobotContainer.kArm;
    protected static final IntakeSub kIntake = RobotContainer.kIntake;

    public AutonomousCommand() {}

    @Override 
    public void end(boolean interrupted) {
        super.end(interrupted);
        kDrivetrain.stopMotor();
        kArm.disable();
        kIntake.disable();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    public abstract Pose2d getInitialPose();

    public abstract void preview();

    public abstract String getName();

}

