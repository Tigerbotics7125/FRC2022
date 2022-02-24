/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSub implements Subsystem {

    static final CANSparkMax m_arm = new CANSparkMax(kId, kMotorType);

    public ArmSub() {}

    public void setArmUp() {
        // set arm to up position
    }

    public void setArmDown() {
        // set arm to down position
    }
}
