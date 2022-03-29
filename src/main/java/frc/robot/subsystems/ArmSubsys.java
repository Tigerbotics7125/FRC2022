/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.Arm.kId;
import static frc.robot.Constants.Arm.kLedLength;
import static frc.robot.Constants.Arm.kNumGradients;
import static frc.robot.Constants.Arm.kSpeed;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the arm of the robot. Forward motor direction results in arm moving up.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ArmSubsys extends SubsystemBase {

    final WPI_TalonSRX mArm = new WPI_TalonSRX(kId);
    final AddressableLED mLeds = new AddressableLED(0);
    int mLedIndex = 0;

    public ArmSubsys() {
        mArm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        mArm.setNeutralMode(NeutralMode.Brake);
        mArm.setInverted(true);

        mLeds.setLength(kLedLength);
        mLeds.start();
    }

    public void disable() {
        mArm.stopMotor();
    }

    public void raise() {
        mArm.set(1 * kSpeed);
    }

    public void lower() {
        mArm.set(-1 * kSpeed);
    }

    public void holdUp() {
        mArm.set(.1);
    }

    public boolean getFwdLimitSwitch() {
        return mArm.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getRevLimitSwitch() {
        return mArm.getSensorCollection().isRevLimitSwitchClosed();
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

    @Override
    public void periodic() {
        AddressableLEDBuffer b = new AddressableLEDBuffer(kLedLength);
        if (RobotState.isDisabled()) {
            // make alliance color and black gradient that winds through the leds when disabled
            for (int i = 0; i < kLedLength; i++) {
                int val =
                        (int)
                                ((255 / 2)
                                        * Math.sin(
                                                (Math.PI
                                                                / (kLedLength * kNumGradients)
                                                                * (i + mLedIndex))
                                                        + (255 / 2)));
                switch (DriverStation.getAlliance()) {
                    case Red:
                        {
                            b.setRGB(i, val, 0, 0);
                            break;
                        }
                    case Blue:
                        {
                            b.setRGB(i, 0, 0, val);
                            break;
                        }
                    case Invalid:
                        {
                        }
                    default:
                        {
                            b.setRGB(i, val, val, val);
                        }
                }
            }
            mLedIndex++;
            mLedIndex = mLedIndex % kLedLength;
        } else {
            if (isUp()) {
                for (int i = 0; i < kLedLength; i++) {
                    b.setRGB(i, 0, 255, 0);
                }
            } else if (isDown()) {
                for (int i = 0; i < kLedLength; i++) {
                    b.setRGB(i, 0, 255, 255);
                }
            } else {
                for (int i = 0; i < kLedLength; i++) {
                    b.setRGB(i, 255, 0, 255);
                }
            }
        }
        mLeds.setData(b);
    }
}