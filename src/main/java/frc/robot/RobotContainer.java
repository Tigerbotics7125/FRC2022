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
import frc.robot.subsystems.ArmSubsys;
import frc.robot.subsystems.ClimberSubsys;
import frc.robot.subsystems.DrivetrainSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.tigerlib.XboxController;

/**
 * Contains and manages all aspects of the robot, and computer.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class RobotContainer {

    // A Chooser, which lets us select between different auto modes, rather than
    // lock one in in code.
    private SendableChooser<Command> mAutoChooser = new SendableChooser<>();
    // Controllers used for driving.
    private XboxController mDriver = new XboxController(0);
    private XboxController mOperator = new XboxController(1);
    // Camera for seeing, duh.
    private UsbCamera mCamera1;
    // Subsystems.
    private DrivetrainSubsys mDrivetrain = new DrivetrainSubsys();
    private ArmSubsys mArm = new ArmSubsys();
    private IntakeSubsys mIntake = new IntakeSubsys();
    private ClimberSubsys mClimber = new ClimberSubsys();

    public RobotContainer() {
        // Add options to the chooser.
        configureAutoChooser();
        // Configure buttons to do things.
        configureDriverButtons();
        configureOperatorButtons();
        // Set up camera; comment out for sim to work.
        configureCameras();

        // Set default subsystem commands.
        // Takes in driver inputs and gives it to the drivetrain so it can go beep boop.
        mDrivetrain.setDefaultCommand(
                new RunCommand(
                                () ->
                                        mDrivetrain.drive(
                                                mDriver.leftX(), mDriver.leftY(), mDriver.rightX()),
                                mDrivetrain)
                        .withName("Default Drive"));
        // Just disable by default.
        mArm.setDefaultCommand(new RunCommand(mArm::disable, mArm).withName("Disable"));
        mIntake.setDefaultCommand(new RunCommand(mIntake::disable, mIntake).withName("Disable"));
        mClimber.setDefaultCommand(new RunCommand(mClimber::disable, mClimber).withName("Disable"));
    }

    /** Update Dashboard values. */
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
        // A no operation command, fancy for sit still and think about what you did.
        mAutoChooser.setDefaultOption(
                "No-op", new InstantCommand(() -> {}).withName("Auto: No Operation"));
        // shoot ball, exit tarmac.
        mAutoChooser.addOption(
                "Reverse Out Of Tarmac", new ExitTarmacCmd(mDrivetrain, mArm, mIntake));
    }

    /** @return The currently selected auto command. */
    public Command getSelectedAuto() {
        return mAutoChooser.getSelected();
    }

    /** Configure buttons for the driver. */
    public void configureDriverButtons() {
        /**
         * Fix any gyro drift, or well technically just tells the gyro what the angle it should
         * treat as forward is.
         *
         * <p>ie: if the robot's "front" was facing directly away from you, all would be correct,
         * however if the "front" of the robot was facing another direction, it will now treat that
         * direction as forward; skewing the controls.
         */
        mDriver.y().whenPressed(new InstantCommand(() -> mDrivetrain.resetGyro()));

        // Toggle field oriented.
        mDriver.rightBumper()
                .whenPressed(
                        new InstantCommand(
                                () ->
                                        mDrivetrain.setFieldOriented(
                                                !mDrivetrain.getFieldOriented())));

        // Toggle heading protection.
        mDriver.leftBumper()
                .whenPressed(
                        new InstantCommand(
                                () ->
                                        mDrivetrain.setHeadingProtection(
                                                !mDrivetrain.getHeadingProtection())));
    }

    /** Configure buttons for the operator. */
    public void configureOperatorButtons() {

        mOperator
                .rightTrigger()
                .whenPressed(new RunCommand(mIntake::eject, mIntake).withName("Eject"))
                .whenReleased(new InstantCommand(mIntake::disable, mIntake).withName("Disable"));

        mOperator
                .leftTrigger()
                .whenPressed(new RunCommand(mIntake::intake, mIntake).withName("Intake"))
                .whenReleased(new InstantCommand(mIntake::disable, mIntake).withName("Disable"));

        mOperator
                .down()
                .whenPressed(
                        new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new RunCommand(mArm::lower, mArm),
                                                new WaitUntilCommand(mArm::isDown),
                                                new WaitCommand(2)),
                                        new RunCommand(mArm::disable, mArm))
                                .withName("Lower Arm Safely"),
                        true);

        mOperator
                .up()
                .whenPressed(
                        new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new RunCommand(mArm::raise, mArm),
                                                new WaitUntilCommand(mArm::isUp),
                                                new WaitCommand(2)),
                                        new RunCommand(
                                                mArm::holdUp,
                                                mArm)) // keeps the arm from falling down.
                                .withName("Raise Arm Safely"),
                        true);

        mOperator
                .leftBumper()
                .whileHeld(new RunCommand(mClimber::rappel).withTimeout(3).withName("Rappel"), true)
                .whenReleased(new RunCommand(mClimber::disable).withName("Disable"), true);

        mOperator
                .rightBumper()
                .whileHeld(new RunCommand(mClimber::winch).withTimeout(3).withName("Winch"), true)
                .whenReleased(new RunCommand(mClimber::disable).withName("Disable"), true);
    }

    public void configureCameras() {
        mCamera1 = CameraServer.startAutomaticCapture();
        mCamera1.setFPS(30);
        mCamera1.setResolution(320, 240);
        mCamera1.setWhiteBalanceAuto();
        mCamera1.setExposureAuto();
        mCamera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    }
}
