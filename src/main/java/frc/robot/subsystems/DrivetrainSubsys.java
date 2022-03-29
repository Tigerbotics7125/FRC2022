/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

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
import static frc.robot.Constants.Drivetrain.kThetaPID;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.tigerlib.Util;

/**
 * Controls the mecanum drivetrain of the robot
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

    // IMU, kinematics, and odometry
    final WPI_PigeonIMU mPigeon = new WPI_PigeonIMU(Constants.kPigeonId);

    final MecanumDriveKinematics mKinematics =
            new MecanumDriveKinematics(
                    kFrontLeftOffset, kFrontRightOffset, kRearLeftOffset, kRearRightOffset);

    final MecanumDriveOdometry mOdometry = new MecanumDriveOdometry(mKinematics, new Rotation2d());

    // Variables used for different driving techniques
    boolean mTurning = true; // whether or not the robot should be turning
    boolean mFieldOriented = true; // whether or not the robot should drive field-oriented
    Rotation2d mDesiredHeading; // the angle to keep the robot facing
    IdleMode currentMode = IdleMode.kCoast; // the current idle mode of the drivetrain

    public DrivetrainSubsys() {
        // set default driving options
        setFieldOriented(false);

        // invert right side because motors backwards.
        mFl.setInverted(false);
        mRl.setInverted(false);
        mFr.setInverted(true);
        mRr.setInverted(true);

        mFlPID.setP(5e-5);
        mRlPID.setP(5e-5);
        mFrPID.setP(5e-5);
        mRrPID.setP(5e-5);

        // changes encoder distance from encoder ticks to meters
        mFlEncoder.setPositionConversionFactor(kDistancePerPulse);
        mRlEncoder.setPositionConversionFactor(kDistancePerPulse);
        mFrEncoder.setPositionConversionFactor(kDistancePerPulse);
        mRrEncoder.setPositionConversionFactor(kDistancePerPulse);

        // changes encoder velocity from rotations per minute to meters per second
        mFlEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        mRlEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        mFrEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);
        mRrEncoder.setVelocityConversionFactor(kRPMtoMPSConversionFactor);

        // make sure stuff starts on 0
        mFlEncoder.setPosition(0.0);
        mRlEncoder.setPosition(0.0);
        mFrEncoder.setPosition(0.0);
        mRrEncoder.setPosition(0.0);
        mPigeon.setFusedHeading(0.0);

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(mFl, DCMotor.getBanebotsRs775(1));
            REVPhysicsSim.getInstance().addSparkMax(mRl, DCMotor.getBanebotsRs775(1));
            REVPhysicsSim.getInstance().addSparkMax(mFr, DCMotor.getBanebotsRs775(1));
            REVPhysicsSim.getInstance().addSparkMax(mRr, DCMotor.getBanebotsRs775(1));
        }
    }

    /** general periodic updates. */
    @Override
    public void periodic() {

        // reset the gyroscope so its square with field on rio user button press
        if (RobotController.getUserButton()) {
            mPigeon.reset();
            mDesiredHeading = getHeading();
        }

        if (RobotState.isDisabled()) {
            // dont have an anurism trying to go back to whatever heading it was at before
            // disabling.
            mDesiredHeading = getHeading();
        }

        // when the robot is disabled put the wheels in coast mode so we can push it
        // around without breaking our ankles
        if (RobotState.isDisabled() && currentMode != IdleMode.kCoast) {
            mFl.setIdleMode(IdleMode.kCoast);
            mRl.setIdleMode(IdleMode.kCoast);
            mFr.setIdleMode(IdleMode.kCoast);
            mRr.setIdleMode(IdleMode.kCoast);
            currentMode = IdleMode.kCoast;
        } else if (!RobotState.isDisabled() && currentMode != IdleMode.kBrake) {
            mFr.setIdleMode(IdleMode.kBrake);
            mRr.setIdleMode(IdleMode.kBrake);
            mFl.setIdleMode(IdleMode.kBrake);
            mRl.setIdleMode(IdleMode.kBrake);
            currentMode = IdleMode.kBrake;
        }

        // setSimHeading(HolonomicTestPath.getInstance().m_thetaPID.getSetpoint().position);
        if (RobotState.isAutonomous()) {
            mOdometry.update(getHeading(), getSpeeds());
        }
        // m_odometry.update(getHeading(), getSpeeds());

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();
        }
    }

    /** reset the odometry of the drivetrain */
    public void resetOdometry(final Pose2d pose) {
        mOdometry.resetPosition(pose, getHeading());
    }

    /** enables or disables turning */
    public void setTurning(boolean turning) {
        mTurning = turning;
    }

    /** enables or disables field oriented driving */
    public void setFieldOriented(boolean fieldOriented) {
        mFieldOriented = fieldOriented;
    }

    /** sets the heading of the robot */
    public void setHeading(Rotation2d heading) {
        mPigeon.setFusedHeading(heading.getDegrees());
    }

    public void toggleFieldOriented() {
        mFieldOriented = !mFieldOriented;
    }

    /**
     * Sets the drivetrain to move as per the given speeds.
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

        // check if we are no longer turning, and set our desired heading to maintain.
        mTurning = zSpeed == 0.0 ? false : true;
        mDesiredHeading = mTurning ? getHeading() : mDesiredHeading;

        // heading protection, keep us facing the same direction.
        if (!mTurning) {
            // negative to get us to go back to the desired orientation, not farther away;
            // that was a fun experience.
            double newSpeed =
                    -kThetaPID.calculate(getHeading().getDegrees(), mDesiredHeading.getDegrees());
            zSpeed = Util.clamp(newSpeed, -.5, .5);
        }

        WheelSpeeds targetSpeeds =
                MecanumDrive.driveCartesianIK(
                        ySpeed, xSpeed, zSpeed, mFieldOriented ? getHeading().getDegrees() : 0.0);

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

    /** @return If turning is enabled. */
    public boolean getTurning() {
        return mTurning;
    }

    /** @return If field oriented driving is enabled. */
    public boolean getFieldOriented() {
        return mFieldOriented;
    }

    /** @return The drivetrains kinematics. */
    public MecanumDriveKinematics getKinematics() {
        return mKinematics;
    }

    /** @return the current heading of the robot */
    public Rotation2d getHeading() {
        // pigeon headings are already +CCW, no need to negate
        return Rotation2d.fromDegrees(mPigeon.getFusedHeading());
    }

    /** @returns the current position of the robot. */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }
}