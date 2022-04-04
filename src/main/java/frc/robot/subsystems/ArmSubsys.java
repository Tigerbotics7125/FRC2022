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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Controls the arm of the robot. Forward motor direction results in arm moving up.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class ArmSubsys extends SubsystemBase {

    // Motor controller for arm.
    final WPI_TalonSRX mArm = new WPI_TalonSRX(kId);

    // LEDs to display arm positions.
    final AddressableLED mLeds = new AddressableLED(0);
    int mLedIndex = 0;

    public ArmSubsys() {
        // Setup encoder.
        mArm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        // Set brake mode to help arm hold positions.
        mArm.setNeutralMode(NeutralMode.Brake);
        // Invert arm so that positive motor voltage results in arm moving up.
        mArm.setInverted(true);

        // Setup LEDs.
        mLeds.setLength(kLedLength);
        mLeds.start();
    }

    /** Disables motor output. */
    public void disable() {
        mArm.stopMotor();
    }

    /** Raises the arm up. */
    public void raise() {
        mArm.set(1 * kSpeed);
    }

    /** Lowers the arm down. */
    public void lower() {
        mArm.set(-1 * kSpeed);
    }

    /** Applies a small amount of power to the arm, but enough to keep it up. */
    public void holdUp() {
        mArm.set(.1);
    }

    /** Gets the forward (up) limit switch's (normally open) state. */
    public Boolean getFwdLimitSwitch() {
        return mArm.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /** Gets the reverse (down) limit switch's (normally closed) state. */
    public Boolean getRevLimitSwitch() {
        return mArm.getSensorCollection().isRevLimitSwitchClosed();
    }

    /** @return if the arm is up. */
    public Boolean isUp() {
        return getFwdLimitSwitch();
    }

    /** @return if the arm is not up. */
    public Boolean isNotUp() {
        return !getFwdLimitSwitch();
    }

    /** @return if the arm is down. */
    public Boolean isDown() {
        return getRevLimitSwitch();
    }

    /** @return if the arm is not down. */
    public Boolean isNotDown() {
        return !getRevLimitSwitch();
    }

    @Override
    public void periodic() {
        // Create a new data packet for the LEDs.
        AddressableLEDBuffer b = new AddressableLEDBuffer(kLedLength);

        // When disabled make a fancy wave with alliance color.
        if (RobotState.isDisabled()) {
            // make alliance color and black gradient that winds through the leds when
            // disabled
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

    /**
     * preforms a self test on the arm, will go up, reset encoder to zero on limit switch, then go
     * down and return the amount of encoder ticks it took to go down.
     *
     * <p>This should not be used on the field as it is only designed for testing and retrieving one
     * time values.
     */
    public Command armSelftTest() {
        return new SequentialCommandGroup(
                new RunCommand(this::raise).deadlineWith(new WaitUntilCommand(() -> isUp())),
                new InstantCommand(() -> mArm.setSelectedSensorPosition(0)),
                new RunCommand(this::lower).deadlineWith(new WaitUntilCommand(() -> isDown())),
                new InstantCommand(
                        () ->
                                SmartDashboard.putNumber(
                                        "Arm Self Test Encoder Value",
                                        mArm.getSelectedSensorPosition())));
    }
}
