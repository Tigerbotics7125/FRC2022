package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.constants.IntakeConstants.*;

public class IntakeSub implements Subsystem {
    
    static final CANSparkMax m_intake = new CANSparkMax(kId, kMotorType);

    public IntakeSub() {
        
    }

    public void intake() {
        m_intake.setVoltage(12);
    }

    public void eject() {
        m_intake.setVoltage(-12);
    }
}
