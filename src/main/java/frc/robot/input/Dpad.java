// Created by Spectrum3847
package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.input.AxisButton.ThresholdType;
import frc.robot.input.XboxGamepad.XboxAxis;
import frc.robot.input.XboxGamepad.XboxDpad;

public class Dpad {
  public final Joystick joy;
  public AxisButton Up;
  public AxisButton Down;
  public AxisButton Left;
  public AxisButton Right;
  public AxisButton UpLeft;
  public AxisButton UpRight;
  public AxisButton DownLeft;
  public AxisButton DownRight;

  public Dpad(Joystick joystick) {
    this.joy = joystick;
    this.Up = new AxisButton(joy, XboxAxis.DPAD, XboxDpad.UP.value, ThresholdType.POV);
    this.Down = new AxisButton(joy, XboxAxis.DPAD, XboxDpad.DOWN.value, ThresholdType.POV);
    this.Left = new AxisButton(joy, XboxAxis.DPAD, XboxDpad.LEFT.value, ThresholdType.POV);
    this.Right = new AxisButton(joy, XboxAxis.DPAD, XboxDpad.RIGHT.value, ThresholdType.POV);
    this.UpLeft = new AxisButton(joy, XboxAxis.DPAD, XboxDpad.UP_LEFT.value, ThresholdType.POV);
    this.UpRight = new AxisButton(joy, XboxAxis.DPAD, XboxDpad.UP_RIGHT.value, ThresholdType.POV);
    this.DownLeft = new AxisButton(joy, XboxAxis.DPAD, XboxDpad.DOWN_LEFT.value, ThresholdType.POV);
    this.DownRight =
        new AxisButton(joy, XboxAxis.DPAD, XboxDpad.DOWN_RIGHT.value, ThresholdType.POV);
  }

  public double getValue() {
    return joy.getRawAxis(XboxAxis.DPAD.value);
  }
}