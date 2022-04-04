/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.Climber.kCurrentLimit;
import static frc.robot.Constants.Climber.kLFollowerId;
import static frc.robot.Constants.Climber.kLId;
import static frc.robot.Constants.Climber.kRFollowerId;
import static frc.robot.Constants.Climber.kRId;
import static frc.robot.Constants.Climber.kSlewRate;
import static frc.robot.Constants.Climber.kSpeed;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the climber on each side of the robot Forward motor direction will winch the the
 * climber.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ClimberSubsys extends SubsystemBase {

    // A single Motor Controller that can control all the motors simultaneously.
    MotorControllerGroup mClimber;
    SlewRateLimiter mRateLimiter = new SlewRateLimiter(kSlewRate);

    // The actual individual motors.
    final WPI_TalonSRX kL = new WPI_TalonSRX(kLId);
    final WPI_TalonSRX kLFollower = new WPI_TalonSRX(kLFollowerId);
    final WPI_TalonSRX kR = new WPI_TalonSRX(kRId);
    final WPI_TalonSRX kRFollower = new WPI_TalonSRX(kRFollowerId);

    public ClimberSubsys() {
        // One motor per side just follows the instructions of its master.
        kLFollower.follow(kL);
        kRFollower.follow(kR);

        // Hold the climber up.
        kL.setNeutralMode(NeutralMode.Brake);
        kLFollower.setNeutralMode(NeutralMode.Brake);
        kR.setNeutralMode(NeutralMode.Brake);
        kRFollower.setNeutralMode(NeutralMode.Brake);

        // Don't kill the motors.
        kL.configContinuousCurrentLimit(kCurrentLimit);
        kLFollower.configContinuousCurrentLimit(kCurrentLimit);
        kR.configContinuousCurrentLimit(kCurrentLimit);
        kRFollower.configContinuousCurrentLimit(kCurrentLimit);

        // The Right side needs to be defaultly inverted, as it is on the opposite side.
        // Each motor needs to be inverted from its master as they are wired backwards.
        kL.setInverted(InvertType.None);
        kLFollower.setInverted(InvertType.OpposeMaster);
        kRFollower.setInverted(InvertType.OpposeMaster);
        kR.setInverted(InvertType.InvertMotorOutput);

        // Init the MCG.
        mClimber = new MotorControllerGroup(kL, kR);
    }

    /** Disables motor output. */
    public void disable() {
        mClimber.stopMotor();
        mRateLimiter.reset(0);
    }

    /** Sets the climber to winch, rope winding under the spool. */
    public void winch() {
        mClimber.set(1 * kSpeed);
    }

    /**
     * Sets the climber to rappel, rope winding over the spool.
     *
     * <p>Should not use this method to complete climb, only to extend the climbers.
     */
    public void rappel() {
        mClimber.set(-1 * kSpeed);
    }
}
