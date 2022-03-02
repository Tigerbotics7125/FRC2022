/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
    // public static final UsbCamera kCamera1 = CameraServer.startAutomaticCapture(0);
    public static final ArmSub kArm = new ArmSub();
    public static final IntakeSub kIntake = new IntakeSub();
}
