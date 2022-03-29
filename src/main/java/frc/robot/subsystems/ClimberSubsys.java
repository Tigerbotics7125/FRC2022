/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.Climber.kLId;
import static frc.robot.Constants.Climber.kMotorType;
import static frc.robot.Constants.Climber.kRId;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the climber on each side of the robot Forward motor direction will winch the the
 * climber.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ClimberSubsys extends SubsystemBase {

    final CANSparkMax mLeft = new CANSparkMax(kLId, kMotorType);
    final CANSparkMax mRight = new CANSparkMax(kRId, kMotorType);

    public ClimberSubsys() {
        mLeft.setInverted(false);
        mRight.setInverted(true);
    }

    public void disable() {
        mLeft.stopMotor();
        mRight.stopMotor();
    }

    public void winch() {
        mLeft.set(1);
        mRight.set(1);
    }

    public void repel() {
        mLeft.set(-1);
        mRight.set(-1);
    }
}
