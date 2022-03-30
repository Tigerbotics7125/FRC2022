/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.ExitTarmacCmd;
import frc.robot.commands.auto.NoOpCmd;
import frc.robot.subsystems.ArmSubsys;
import frc.robot.subsystems.ClimberSubsys;
import frc.robot.subsystems.DrivetrainSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.tigerlib.XboxController;

/**
 * Contains and manages subsystems of the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class RobotContainer {

    private SendableChooser<Command> mAutoChooser = new SendableChooser<>();
    // private GExtreme3DProJoystick mFliJoy = new GExtreme3DProJoystick(0);
    private XboxController mDriver = new XboxController(0);
    private XboxController mOperator = new XboxController(1);
    private UsbCamera mCamera1;
    // public final PowerDistribution mPdp = new PowerDistribution(0,
    // ModuleType.kCTRE);
    private DrivetrainSubsys mDrivetrain = new DrivetrainSubsys();
    private ArmSubsys mArm = new ArmSubsys();
    private IntakeSubsys mIntake = new IntakeSubsys();
    private ClimberSubsys mClimber = new ClimberSubsys();

    public RobotContainer() {
        configureAutoChooser();
        configureButtonBindings();
        configureCameras();

        // set default commands
        mDrivetrain.setDefaultCommand(
                new RunCommand(
                                () ->
                                        mDrivetrain.drive(
                                                /*mFliJoy.xAxis()*/
                                                mDriver.leftX(),
                                                /*mFliJoy.yAxis()*/
                                                mDriver.leftY(),
                                                /*mFliJoy.zAxis()*/
                                                mDriver.rightX()),
                                mDrivetrain)
                        .withName("Default Drive"));
        mArm.setDefaultCommand(new RunCommand(mArm::disable, mArm).withName("Disable"));
        mIntake.setDefaultCommand(new RunCommand(mIntake::disable, mIntake).withName("Disable"));
        mClimber.setDefaultCommand(new RunCommand(mClimber::disable, mClimber).withName("Disable"));
    }

    public void updateValues() {

        // Auto
        SmartDashboard.putData("AutoChooser", mAutoChooser);
        if (mAutoChooser.getSelected() != null) {
            SmartDashboard.putString("Auto To Run", mAutoChooser.getSelected().getName());
        }

        // Robot Info
        // SmartDashboard.putNumber("Heading", mDrivetrain.getHeading().getDegrees());
        SmartDashboard.putBoolean("Is up?", mArm.getFwdLimitSwitch());
        SmartDashboard.putBoolean("Is down?", mArm.getRevLimitSwitch());
        SmartDashboard.putBoolean("Heading Protection?", mDrivetrain.getHeadingProtection());
        SmartDashboard.putBoolean("Field Oriented?", mDrivetrain.getFieldOriented());

        // Drivetrain headings.
        SmartDashboard.putNumber("Current Heading", mDrivetrain.getHeading().getDegrees());
        SmartDashboard.putNumber("Desired Heading", mDrivetrain.getDesiredHeading().getDegrees());

        // Subsystems
        // SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
        // SmartDashboard.putData("PDP", RobotContainer.kPdp);;
        SmartDashboard.putData("Drivetrain", (SubsystemBase) mDrivetrain);
        SmartDashboard.putData("Arm", (SubsystemBase) mArm);
        SmartDashboard.putData("Intake", (SubsystemBase) mIntake);
        SmartDashboard.putData("Climber", (SubsystemBase) mClimber);

        // Puts all sendable data to the dashboard.
        SmartDashboard.updateValues();
    }

    public void configureAutoChooser() {
        mAutoChooser.setDefaultOption("No-op", new NoOpCmd(mDrivetrain, mArm, mIntake));
        mAutoChooser.addOption(
                "Reverse Out Of Tarmac", new ExitTarmacCmd(mDrivetrain, mArm, mIntake));
    }

    public Command getSelectedAuto() {
        return mAutoChooser.getSelected();
    }

    public void configureButtonBindings() {

        // pressing the trigger ejects the ball
        /*mFliJoy.trigger()*/
        mDriver.rightTrigger()
                .whenPressed(new RunCommand(mIntake::eject, mIntake).withName("Eject"))
                .whenReleased(new InstantCommand(mIntake::disable, mIntake).withName("Disable"));

        // pressing the thumb button intakes the balls
        /*mFliJoy.thumb()*/
        mDriver.leftTrigger()
                .whenPressed(new RunCommand(mIntake::intake, mIntake).withName("Intake"))
                .whenReleased(new InstantCommand(mIntake::disable, mIntake).withName("Disable"));

        // pressing button 3 lowers the arm safely
        /*mFliJoy.three()*/
        mDriver.down()
                .whenPressed(
                        new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new RunCommand(mArm::lower, mArm),
                                                new WaitUntilCommand(mArm::isDown),
                                                new WaitCommand(2)),
                                        new RunCommand(mArm::disable, mArm))
                                .withName("Lower Arm Safely"),
                        true);

        // pressing button 5 raises the arm safely
        /*mFliJoy.five()*/
        mDriver.up()
                .whenPressed(
                        new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new RunCommand(mArm::raise, mArm),
                                                new WaitUntilCommand(mArm::isUp),
                                                new WaitCommand(2)),
                                        new RunCommand(mArm::holdUp, mArm))
                                .withName("Raise Arm Safely"),
                        true);

        /*mFliJoy.twelve()*/
        mDriver.leftBumper()
                .whileHeld(new RunCommand(mClimber::rappel).withTimeout(3).withName("Rappel"), true)
                .whenReleased(new RunCommand(mClimber::disable).withName("Disable"), true);

        /*mFliJoy.eleven()*/
        mDriver.rightBumper()
                .whileHeld(new RunCommand(mClimber::winch).withTimeout(3).withName("Winch"), true)
                .whenReleased(new RunCommand(mClimber::disable).withName("Disable"), true);
    }

    public void configureCameras() {
        mCamera1 = CameraServer.startAutomaticCapture();
        mCamera1.setFPS(30);
        mCamera1.setResolution(320 / 32, 240 / 32);
        mCamera1.setWhiteBalanceAuto();
        mCamera1.setExposureAuto();
        mCamera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    }
}
