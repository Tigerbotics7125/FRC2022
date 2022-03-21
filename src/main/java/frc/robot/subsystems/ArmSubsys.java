/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.Arm.kId;
import static frc.robot.Constants.Arm.kSpeed;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the arm of the robot. Forward motor direction results in arm moving up.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ArmSubsys extends SubsystemBase {

    final WPI_TalonSRX m_arm = new WPI_TalonSRX(kId);

    public ArmSubsys() {
        m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        m_arm.setNeutralMode(NeutralMode.Brake);
        m_arm.setInverted(true);
    }

    public void disable() {
        m_arm.stopMotor();
    }

    public void raise() {
        m_arm.set(1 * kSpeed);
    }

    public void lower() {
        m_arm.set(-1 * kSpeed);
    }

    public boolean getFwdLimitSwitch() {
        return m_arm.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getRevLimitSwitch() {
        return m_arm.getSensorCollection().isRevLimitSwitchClosed();
    }

    public Boolean isUp() {
        return getFwdLimitSwitch();
    }

    public Boolean isNotUp() {
        return !getFwdLimitSwitch();
    }

    public Boolean isDown() {
        return getRevLimitSwitch();
    }

    public Boolean isNotDown() {
        return !getRevLimitSwitch();
    }
}
