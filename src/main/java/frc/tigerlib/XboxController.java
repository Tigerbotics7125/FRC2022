/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.tigerlib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * An input wrapper for the Xbox controller.
 *
 * <p>Saves RAM on the roborio by only instantiating a button when its used. CPU time is cheaper
 * than RAM in this scenario.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class XboxController extends GenericHID {

    /* Represents an analog axis on the joystick. */
    public enum Axis {
        kLeftX(0),
        kLeftY(1),
        kLT(2),
        kRT(3),
        kRightX(4),
        kRightY(5);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    /* Represents a digital button on the joystick. */
    public enum Button {
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        kLB(5),
        kRB(6),
        kBack(7),
        kStart(8),
        kLStick(9),
        kRStick(10);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    private JoystickButton _aButton;
    private JoystickButton _bButton;
    private JoystickButton _xButton;
    private JoystickButton _yButton;
    private JoystickButton _lbButton;
    private JoystickButton _rbButton;
    private JoystickButton _backButton;
    private JoystickButton _startButton;
    private JoystickButton _lStickButton;
    private JoystickButton _rStickButton;
    private AxisButton _lt;
    private AxisButton _rt;
    private POVButton _uButton;
    private POVButton _urButton;
    private POVButton _rButton;
    private POVButton _drButton;
    private POVButton _dButton;
    private POVButton _dlButton;
    private POVButton _lButton;
    private POVButton _ulButton;

    /**
     * Constructs a new instance of the joystick
     *
     * @param port The port on the Driver Station that the joystick is plugged into
     */
    public XboxController(int port) {
        super(port);
    }

    /**
     * Returns the a button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton a() {
        if (_aButton == null) {
            _aButton = new JoystickButton(this, Button.kA.value);
        }
        return _aButton;
    }

    /**
     * Returns the b button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton b() {
        if (_bButton == null) {
            _bButton = new JoystickButton(this, Button.kB.value);
        }
        return _bButton;
    }

    /**
     * Returns the x button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton x() {
        if (_xButton == null) {
            _xButton = new JoystickButton(this, Button.kX.value);
        }
        return _xButton;
    }

    /**
     * Returns the y button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton y() {
        if (_yButton == null) {
            _yButton = new JoystickButton(this, Button.kY.value);
        }
        return _yButton;
    }

    /**
     * Returns the left bumper button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton leftBumper() {
        if (_lbButton == null) {
            _lbButton = new JoystickButton(this, Button.kLB.value);
        }
        return _lbButton;
    }

    /**
     * Returns the right bumper button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton rightBumper() {
        if (_rbButton == null) {
            _rbButton = new JoystickButton(this, Button.kRB.value);
        }
        return _rbButton;
    }

    /**
     * Returns the back button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton back() {
        if (_backButton == null) {
            _backButton = new JoystickButton(this, Button.kBack.value);
        }
        return _backButton;
    }

    /**
     * Returns the start button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton start() {
        if (_startButton == null) {
            _startButton = new JoystickButton(this, Button.kStart.value);
        }
        return _startButton;
    }

    /**
     * Returns the left stick button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton lStick() {
        if (_lStickButton == null) {
            _lStickButton = new JoystickButton(this, Button.kLStick.value);
        }
        return _lStickButton;
    }

    /**
     * Returns the right stick button's {@link JoystickButton}.
     *
     * <p>to get its value, use {@link JoystickButton#get()}.
     */
    public JoystickButton rStick() {
        if (_rStickButton == null) {
            _rStickButton = new JoystickButton(this, Button.kRStick.value);
        }
        return _rStickButton;
    }

    /**
     * Returns the up button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton up() {
        if (_uButton == null) {
            _uButton = new POVButton(this, 0);
        }
        return _uButton;
    }

    /**
     * Returns the up-right button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton upRight() {
        if (_urButton == null) {
            _urButton = new POVButton(this, 45);
        }
        return _urButton;
    }

    /**
     * Returns the right button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton right() {
        if (_rButton == null) {
            _rButton = new POVButton(this, 90);
        }
        return _rButton;
    }

    /**
     * Returns the down-right button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton downRight() {
        if (_drButton == null) {
            _drButton = new POVButton(this, 135);
        }
        return _drButton;
    }

    /**
     * Returns the down button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton down() {
        if (_dButton == null) {
            _dButton = new POVButton(this, 180);
        }
        return _dButton;
    }

    /**
     * Returns the down-left button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton downLeft() {
        if (_dlButton == null) {
            _dlButton = new POVButton(this, 225);
        }
        return _dlButton;
    }

    /**
     * Returns the left button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton left() {
        if (_lButton == null) {
            _lButton = new POVButton(this, 270);
        }
        return _lButton;
    }

    /**
     * Returns the up-left button's {@link POVButton}.
     *
     * <p>to get its value, use {@link POVButton#get()}.
     */
    public POVButton upLeft() {
        if (_ulButton == null) {
            _ulButton = new POVButton(this, 315);
        }
        return _ulButton;
    }

    /**
     * Get the X value of the left stick.
     *
     * <p>Right is positive.
     *
     * @reutrn the X value of the joystick.
     */
    public double leftX() {
        return getRawAxis(Axis.kLeftX.value);
    }

    /**
     * Get the Y value of the left stick.
     *
     * <p>Forward is positive.
     *
     * @reutrn the Y value of the joystick.
     */
    public double leftY() {
        return -getRawAxis(Axis.kLeftY.value);
    }

    /**
     * Get the X value of the right stick.
     *
     * <p>Right is positive.
     *
     * @reutrn the X value of the joystick.
     */
    public double rightX() {
        return getRawAxis(Axis.kRightX.value);
    }

    /**
     * Get the Y value of the right stick.
     *
     * <p>Forward is positive.
     *
     * @reutrn the Y value of the joystick.
     */
    public double rightY() {
        return -getRawAxis(Axis.kRightY.value);
    }

    /**
     * Get the left trigger's {@link AxisButton}
     *
     * <p>to get its value, use {@link AxisButton#get()}.
     */
    public AxisButton leftTrigger() {
        if (_lt == null) {
            _lt = new AxisButton(this, Axis.kLT.value, .3);
        }
        return _lt;
    }

    /**
     * Get the Right Trigger value.
     *
     * <p>Not pressed is 0, down is 1.
     *
     * @return the Right Trigger value.
     */
    public AxisButton rightTrigger() {
        if (_rt == null) {
            _rt = new AxisButton(this, Axis.kRT.value, .3);
        }
        return _rt;
    }
}
