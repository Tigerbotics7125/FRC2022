/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;

public class NothingAuto extends AutonomousCommand {

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d();
    }

    @Override
    public void preview() {}

    @Override
    public String getName() {
        return "Do Nothing";
    }
}
