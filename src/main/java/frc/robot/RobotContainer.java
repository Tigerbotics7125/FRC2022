/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot;

import static frc.robot.Constants.Drivetrain.kDeadband;
import static frc.robot.Constants.Drivetrain.kSensitivity;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.ExitTarmacCmd;
import frc.robot.commands.auto.NoOpCmd;
import frc.robot.subsystems.ArmSubsys;
import frc.robot.subsystems.ClimberSubsys;
import frc.robot.subsystems.DrivetrainSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.LEDSubsys;
import frc.tigerlib.GExtreme3DProJoystick;
import frc.tigerlib.Util;

;

/**
 * Contains and manages subsystems of the robot.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class RobotContainer {

    private SendableChooser<Command> mAutoChooser = new SendableChooser<>();
    private GExtreme3DProJoystick mJoystick = new GExtreme3DProJoystick(0);
    private UsbCamera mCamera1;
    // public final PowerDistribution mPdp = new PowerDistribution(0,
    // ModuleType.kCTRE);
    private DrivetrainSubsys mDrivetrain = new DrivetrainSubsys();
    private ArmSubsys mArm = new ArmSubsys();
    private IntakeSubsys mIntake = new IntakeSubsys();
    private ClimberSubsys mClimber = new ClimberSubsys();
    private LEDSubsys mLeds = new LEDSubsys();

    public RobotContainer() {
        configureAutoChooser();
        configureButtonBindings();
        configureCameras();

        // set default commands
        mDrivetrain.setDefaultCommand(
                new RunCommand(
                                () ->
                                        mDrivetrain.drive(
                                                mJoystick.xAxis(),
                                                mJoystick.yAxis(),
                                                mJoystick.zAxis()),
                                mDrivetrain)
                        .withName("Default Drive"));
        mArm.setDefaultCommand(new RunCommand(mArm::disable, mArm).withName("Disable"));
        mIntake.setDefaultCommand(new RunCommand(mIntake::disable, mIntake).withName("Disable"));
        mClimber.setDefaultCommand(new RunCommand(mClimber::disable, mClimber).withName("Disable"));
    }

    public void updateValues() {
        // Controller
        SmartDashboard.putNumber(
                "xInput",
                Util.scaledDeadbandClamp(mJoystick.xAxis(), kDeadband, kSensitivity, -1, 1));
        SmartDashboard.putNumber(
                "yInput",
                Util.scaledDeadbandClamp(mJoystick.yAxis(), kDeadband, kSensitivity, -1, 1));
        SmartDashboard.putNumber(
                "zInput",
                Util.scaledDeadbandClamp(mJoystick.zAxis(), kDeadband, kSensitivity, -1, 1));
        SmartDashboard.putNumber(
                "tInput",
                Util.scaledDeadbandClamp(mJoystick.throttleAxis(), kDeadband, kSensitivity, -1, 1));

        // Auto
        SmartDashboard.putData("Auto", mAutoChooser);
        SmartDashboard.putString("AutoChooser", mAutoChooser.getSelected().getName());

        // Robot Info
        // SmartDashboard.putNumber("Heading", mDrivetrain.getHeading().getDegrees());
        SmartDashboard.putBoolean("Is up?", mArm.getFwdLimitSwitch());
        SmartDashboard.putBoolean("Is down?", mArm.getRevLimitSwitch());

        // Driving Options
        // SmartDashboard.putBoolean("Turning?", mDrivetrain.getTurning());
        // SmartDashboard.putBoolean("Field Oriented?", mDrivetrain.getFieldOriented());

        // Subsystems
        // SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
        // SmartDashboard.putData("PDP", RobotContainer.kPdp);;
        SmartDashboard.putData("Drivetrain", mDrivetrain);
        SmartDashboard.putData("Arm", mArm);
        SmartDashboard.putData("Intake", mIntake);
        SmartDashboard.putData("Climber", mClimber);

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
        mJoystick
                .trigger()
                .whenPressed(new RunCommand(mIntake::eject, mIntake).withName("Eject"))
                .whenReleased(new InstantCommand(mIntake::disable, mIntake).withName("Disable"));

        // pressing the thumb button intakes the balls
        mJoystick
                .thumb()
                .whenPressed(new RunCommand(mIntake::intake, mIntake).withName("Intake"))
                .whenReleased(new InstantCommand(mIntake::disable, mIntake).withName("Disable"));

        // pressing button 3 lowers the arm safely
        mJoystick
                .three()
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
        mJoystick
                .five()
                .whenPressed(
                        new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new RunCommand(mArm::raise, mArm),
                                                new WaitUntilCommand(mArm::isUp),
                                                new WaitCommand(2)),
                                        new RunCommand(mArm::disable, mArm))
                                .withName("Raise Arm Safely"),
                        true);

        mJoystick
                .twelve()
                .whileHeld(new RunCommand(mClimber::rappel).withTimeout(3).withName("Rappel"), true)
                .whenReleased(new RunCommand(mClimber::disable).withName("Disable"), true);

        mJoystick
                .eleven()
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
