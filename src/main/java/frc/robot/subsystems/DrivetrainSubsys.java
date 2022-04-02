/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.Drivetrain.kDeadband;
import static frc.robot.Constants.Drivetrain.kDistancePerPulse;
import static frc.robot.Constants.Drivetrain.kFrontLeftId;
import static frc.robot.Constants.Drivetrain.kFrontLeftOffset;
import static frc.robot.Constants.Drivetrain.kFrontRightId;
import static frc.robot.Constants.Drivetrain.kFrontRightOffset;
import static frc.robot.Constants.Drivetrain.kMotorType;
import static frc.robot.Constants.Drivetrain.kRPMtoMPSConversionFactor;
import static frc.robot.Constants.Drivetrain.kRearLeftId;
import static frc.robot.Constants.Drivetrain.kRearLeftOffset;
import static frc.robot.Constants.Drivetrain.kRearRightId;
import static frc.robot.Constants.Drivetrain.kRearRightOffset;
import static frc.robot.Constants.Drivetrain.kSensitivity;
import static frc.robot.Constants.Drivetrain.kXSlewRate;
import static frc.robot.Constants.Drivetrain.kYSlewRate;
import static frc.robot.Constants.Drivetrain.kZPID;
import static frc.robot.Constants.Drivetrain.kZSlewRate;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.tigerlib.Util;

/**
 * Controls the mecanum drivetrain of the robot.
 *
 * <p>This is the most advanded subsystem of the robot; comprising PID control, odometry, heading
 * protection, rate limiters, and field oriented driving strategies.
 *
 * @author 7125 Tigerbotics - Jeffrey Morris
 */
public class DrivetrainSubsys extends SubsystemBase {

    // Motors, PID controllers, and encoders
    final CANSparkMax mFl = new CANSparkMax(kFrontLeftId, kMotorType);
    final CANSparkMax mRl = new CANSparkMax(kRearLeftId, kMotorType);
    final CANSparkMax mFr = new CANSparkMax(kFrontRightId, kMotorType);
    final CANSparkMax mRr = new CANSparkMax(kRearRightId, kMotorType);

    final SparkMaxPIDController mFlPID = mFl.getPIDController();
    final SparkMaxPIDController mRlPID = mRl.getPIDController();
    final SparkMaxPIDController mFrPID = mFr.getPIDController();
    final SparkMaxPIDController mRrPID = mRr.getPIDController();

    final RelativeEncoder mFlEncoder = mFl.getEncoder();
    final RelativeEncoder mRlEncoder = mRl.getEncoder();
    final RelativeEncoder mFrEncoder = mFr.getEncoder();
    final RelativeEncoder mRrEncoder = mRr.getEncoder();

    final SlewRateLimiter mXSlew = new SlewRateLimiter(kXSlewRate);
    final SlewRateLimiter mYSlew = new SlewRateLimiter(kYSlewRate);
    final SlewRateLimiter mZSlew = new SlewRateLimiter(kZSlewRate);

    // Pigeon gyroscope.
    final WPI_PigeonIMU mPigeon = new WPI_PigeonIMU(Constants.kPigeonId);

    // Drivetrain math, allows for finding speeds of chassis from wheels and vise
    // versa.
    final MecanumDriveKinematics mKinematics =
            new MecanumDriveKinematics(
                    kFrontLeftOffset, kFrontRightOffset, kRearLeftOffset, kRearRightOffset);

    // Tracks the position of the robot, based on encoder values.
    final MecanumDriveOdometry mOdometry = new MecanumDriveOdometry(mKinematics, new Rotation2d());

    // Variables used for different driving techniques
    boolean mHeadingProtect =
            true; // whether or not the robot should be maintaining its desired heading.
    boolean mFieldOriented = true; // whether or not the robot should drive field-oriented
    boolean mCapturedHeading =
            false; // whether or not the robot has captured its desired heading yet.
    Rotation2d mDesiredHeading = getHeading(); // the angle to keep the robot facing
    // The command to capture the desired heading, basically just waits a little bit
    // after moving so it doesnt freak out.
    Command mCaptureHeadingCmd =
            new SequentialCommandGroup(
                    new WaitCommand(.4),
                    new InstantCommand(() -> mDesiredHeading = getHeading()),
                    new InstantCommand(() -> mCapturedHeading = true));
    IdleMode mCurrMode = IdleMode.kBrake; // the current idle mode of the drivetrain

    public DrivetrainSubsys() {

        // Set up safe amperage limits.
        mFl.setSmartCurrentLimit(50);
        mFr.setSmartCurrentLimit(50);
        mRl.setSmartCurrentLimit(50);
        mRr.setSmartCurrentLimit(50);

        // Invert right side because motors backwards.
        mFl.setInverted(false);
        mRl.setInverted(false);
        mFr.setInverted(true);
        mRr.setInverted(true);

        // Setup PID controllers.
        mFlPID.setP(5e-5);
        mRlPID.setP(5e-5);
        mFrPID.setP(5e-5);
        mRrPID.setP(5e-5);

        // Changes encoder distance from encoder ticks to meters.
        mFlEncoder.setPositionConversionFactor(kDistancePerPulse);
        mRlEncoder.setPositionConversionFactor(kDistancePerPulse);
        mFrEncoder.setPositionConversionFactor(kDistancePerPulse);
        mRrEncoder.setPositionConversionFactor(kDistancePerPulse);

        // Changes encoder velocity from rotations per minute to meters per second.
        mFlEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        mRlEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        mFrEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        mRrEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);

        // Make sure stuff starts on 0.
        mFlEncoder.setPosition(0.0);
        mRlEncoder.setPosition(0.0);
        mFrEncoder.setPosition(0.0);
        mRrEncoder.setPosition(0.0);
        mPigeon.setFusedHeading(0.0);

        // In sim add motors to physics sim.
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(mFl, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(mRl, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(mFr, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(mRr, DCMotor.getNEO(1));
        }
    }

    /** general periodic updates. */
    @Override
    public void periodic() {

        // Reset the gyroscope so its square with field on rio user button press.
        if (RobotController.getUserButton()) {
            resetGyro();
        }

        // Set desired heading so if we move robot while disabled it wont kill us.
        if (RobotState.isDisabled()) {
            // dont have an anurism trying to go back to whatever heading it was at before
            // disabling.
            mDesiredHeading = getHeading();
        }

        // when the robot is disabled put the wheels in coast mode so we can push it
        // around without breaking our ankles
        if (RobotState.isDisabled() && mCurrMode != IdleMode.kCoast) {
            mFl.setIdleMode(IdleMode.kCoast);
            mRl.setIdleMode(IdleMode.kCoast);
            mFr.setIdleMode(IdleMode.kCoast);
            mRr.setIdleMode(IdleMode.kCoast);
            mCurrMode = IdleMode.kCoast;
        } else if (!RobotState.isDisabled() && mCurrMode != IdleMode.kBrake) {
            mFr.setIdleMode(IdleMode.kBrake);
            mRr.setIdleMode(IdleMode.kBrake);
            mFl.setIdleMode(IdleMode.kBrake);
            mRl.setIdleMode(IdleMode.kBrake);
            mCurrMode = IdleMode.kBrake;
        }

        if (RobotState.isAutonomous()) {
            // only tract robot during auto as its fairly computationally expensive.
            mOdometry.update(getHeading(), getSpeeds());
        }

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();
        }
    }

    /** reset the odometry of the drivetrain */
    public void resetOdometry(final Pose2d pose) {
        mOdometry.resetPosition(pose, getHeading());
    }

    /** Sets the heading protection status. */
    public void setHeadingProtection(boolean headingProtection) {
        mHeadingProtect = headingProtection;
    }

    /** Sets the field oriented status. */
    public void setFieldOriented(boolean fieldOriented) {
        mFieldOriented = fieldOriented;
    }

    /** Resets the gyro to zero. */
    public void resetGyro() {
        mPigeon.reset();
        mDesiredHeading = getHeading();
    }

    /**
     * Sets the drivetrain to move as per the given speeds.
     *
     * <p>Use meters per second.
     *
     * @param targetSpeeds The input speeds.
     */
    public void setSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
        mFlPID.setReference(targetSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
        mRlPID.setReference(targetSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
        mFrPID.setReference(targetSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
        mRrPID.setReference(targetSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
    }

    /**
     * Drives the robot based on the input. All input is clamped to [-1, 1]; input should be scaled
     * before passing into this method.
     *
     * @param xSpeed Robot X Speed, forward is positive.
     * @param ySpeed Robot Y Speed, Right is positive.
     * @param zSpeed Robot Z/Theta Speed, Clockwise is positive.
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        xSpeed = Util.scaledDeadbandClamp(mXSlew.calculate(xSpeed), kDeadband, kSensitivity, -1, 1);
        ySpeed = Util.scaledDeadbandClamp(mYSlew.calculate(ySpeed), kDeadband, kSensitivity, -1, 1);
        zSpeed = Util.scaledDeadbandClamp(mZSlew.calculate(zSpeed), kDeadband, kSensitivity, -1, 1);

        // heading protection, keep us facing the same direction.
        boolean shouldProtectHeading = mHeadingProtect && zSpeed == 0.0;
        if (shouldProtectHeading && mCapturedHeading) {
            // if we should protect heading and we have captured the desired heading

            // negative to get us to go back to the desired orientation, not farther away;
            // that was a fun experience.
            double newSpeed =
                    -kZPID.calculate(getHeading().getDegrees(), mDesiredHeading.getDegrees());
            zSpeed = Util.clamp(newSpeed, -.75, .75);
        } else if (shouldProtectHeading
                && !CommandScheduler.getInstance().isScheduled(mCaptureHeadingCmd)) {
            // if we should protect heading and we havnt started to capture desired heading
            CommandScheduler.getInstance().schedule(mCaptureHeadingCmd);
        } else if (!shouldProtectHeading) {
            // if we should not protect heading
            mCapturedHeading = false;
            mDesiredHeading = getHeading();
        }

        // Do some beep boop to get wheel speeds.
        // Heading is negated as Drive classes are NED >:(, will be fixed 2023.
        WheelSpeeds targetSpeeds =
                MecanumDrive.driveCartesianIK(
                        ySpeed, xSpeed, zSpeed, mFieldOriented ? -getHeading().getDegrees() : 0.0);

        // Set the speeds, use PID controllers for consistency.
        mFlPID.setReference(targetSpeeds.frontLeft, ControlType.kDutyCycle);
        mRlPID.setReference(targetSpeeds.rearLeft, ControlType.kDutyCycle);
        mFrPID.setReference(targetSpeeds.frontRight, ControlType.kDutyCycle);
        mRrPID.setReference(targetSpeeds.rearRight, ControlType.kDutyCycle);
    }

    /** Disables all motor output */
    public void disable() {
        mFl.disable();
        mRl.disable();
        mFr.disable();
        mRr.disable();
    }

    /** @return The current velocity of the robot. */
    public MecanumDriveWheelSpeeds getSpeeds() {
        return new MecanumDriveWheelSpeeds(
                mFlEncoder.getVelocity(),
                mRlEncoder.getVelocity(),
                mFrEncoder.getVelocity(),
                mRrEncoder.getVelocity());
    }

    /** @return Heading protections status. */
    public boolean getHeadingProtection() {
        return mHeadingProtect;
    }

    /** @return Field oriented status. */
    public boolean getFieldOriented() {
        return mFieldOriented;
    }

    /** @return The drivetrains kinematics. */
    public MecanumDriveKinematics getKinematics() {
        return mKinematics;
    }

    /**
     * Returns the heading; CCW+.
     *
     * @return the current heading of the robot
     */
    public Rotation2d getHeading() {
        return mPigeon.getRotation2d();
    }

    /** @return the current desired heading of the robot */
    public Rotation2d getDesiredHeading() {
        return mDesiredHeading;
    }

    /** @returns the current position of the robot. */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }
}
