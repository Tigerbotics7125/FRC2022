/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

import static frc.robot.constants.MecanumDrivetrainConstants.kMaxAutoAcceleration;
import static frc.robot.constants.MecanumDrivetrainConstants.kMaxAutoVelocity;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousTrajectories {
    public static final PathPlannerTrajectory[] kHolonomicTestPath = {
        PathPlanner.loadPath("HolonomicTestPath", kMaxAutoVelocity, kMaxAutoAcceleration)
    };

    public static final PathPlannerTrajectory[] kTwoBallAuto = {
        PathPlanner.loadPath("2BallAuto", kMaxAutoVelocity, kMaxAutoAcceleration)
    };
}
