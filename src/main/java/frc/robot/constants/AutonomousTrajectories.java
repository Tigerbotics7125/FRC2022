/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.constants;

import static frc.robot.constants.MecanumDrivetrainConstants.kMaxAutoAcceleration;
import static frc.robot.constants.MecanumDrivetrainConstants.kMaxAutoVelocity;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * Contains all autonomous trajectories.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class AutonomousTrajectories {

    public static final PathPlannerTrajectory[] kHolonomicTestPath = {
        PathPlanner.loadPath("HolonomicTestPath", kMaxAutoVelocity, kMaxAutoAcceleration)
    };

    public static final PathPlannerTrajectory[] kOneBallTarmacExit = {
        PathPlanner.loadPath("1BallTarmacExit", kMaxAutoVelocity, kMaxAutoAcceleration)
    };

    public static final PathPlannerTrajectory[] kTwoBallTarmacExit = {
        PathPlanner.loadPath("2BallTarmacExit", kMaxAutoVelocity, kMaxAutoAcceleration)
    };

    public static final PathPlannerTrajectory[] kTwoBallAuto = {
        PathPlanner.loadPath("2BallAuto", kMaxAutoVelocity, kMaxAutoAcceleration)
    };

    public static final PathPlannerTrajectory[] kFiveBallAuto = {
        PathPlanner.loadPath("5BallAuto", kMaxAutoAcceleration, kMaxAutoAcceleration)
    };
}
