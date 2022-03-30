package frc.tigerlib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} class for axis's on a {@link GenericHID}.
 * 
 * <p> the button will be declared as pressed when the axis is greater than the threshold.
 * 
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class AxisButton extends Button {
    private GenericHID mJoystick;
    private int mAxis;
    private double mThreshold;

    public AxisButton(GenericHID joystick, int axis, double threshold) {
        mJoystick = joystick;
        mAxis = axis;
        mThreshold = threshold;
    }

    @Override
    public boolean get() {
        if (mJoystick.getRawAxis(mAxis) > mThreshold) {
            return true;
        } else {
            return false;
        }
    }
}
