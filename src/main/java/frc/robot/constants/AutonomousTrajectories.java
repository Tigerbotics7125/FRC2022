/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousTrajectories {
    public static final PathPlannerTrajectory[] kHolonomicTestPath = {
        PathPlanner.loadPath(
                "HolonomicTestPath",
                MecanumDrivetrainConstants.kMaxAutoVelocity,
                MecanumDrivetrainConstants.kMaxAutoAcceleration)
    };
}
