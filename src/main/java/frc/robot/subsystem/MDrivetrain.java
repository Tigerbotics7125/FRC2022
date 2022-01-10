package frc.robot.subsystem;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MDrivetrain extends SubsystemBase {

    private TalonSRX flMotor = new TalonSRX(0);
    private TalonSRX frMotor = new TalonSRX(2);
    private TalonSRX rlMotor = new TalonSRX(1);
    private TalonSRX rrMotor = new TalonSRX(3);
    private PigeonIMU pidgey = new PigeonIMU(new TalonSRX(4));
    private double[] xyz_dps = new double[3];

    // TODO: measure the distance from physical center of robot to each wheel
    // ROBOT FACES POSITIVE X DIRECTION
    // Use negative values when needed, based on cordinate plane not distances.
    private Translation2d fl = new Translation2d(/* x */ Units.inchesToMeters(0.0), /* y */ Units.inchesToMeters(0.0));
    private Translation2d fr = new Translation2d(/* x */ Units.inchesToMeters(0.0), /* y */ Units.inchesToMeters(0.0));
    private Translation2d rl = new Translation2d(/* x */ Units.inchesToMeters(0.0), /* y */ Units.inchesToMeters(0.0));
    private Translation2d rr = new Translation2d(/* x */ Units.inchesToMeters(0.0), /* y */ Units.inchesToMeters(0.0));
    private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(fl, fr, rl, rr);

    private MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, getHeading());

    private Pose2d pose = new Pose2d();

    /*
     * Set constant vairiables, so that they could easily be put to SmartDashboard
     * and tinkered with
     */
    private double kp = 9.95;
    private double ki = 0.0;
    private double kd = 0.0;
    private double maxTuringVelocity_mps = 5.0;
    private double maxTurningAcceleration_mps = 5.0;
    // use frc characterization tool for these
    private double ks = 0.0; // static gain
    private double kv = 0.0; // velocity gain
    private double ka = 0.0; // acceleration gain

    private PIDController xController = new PIDController(kp, ki, kd);
    private PIDController yController = new PIDController(kp, ki, kd);
    private Constraints thetaConstraints = new Constraints(maxTuringVelocity_mps, maxTurningAcceleration_mps);
    private ProfiledPIDController thetaController = new ProfiledPIDController(kp, ki, kd, thetaConstraints);

    private PIDController flPIDController = new PIDController(kp, ki, kd);
    private PIDController frPIDController = new PIDController(kp, ki, kd);
    private PIDController rlPIDController = new PIDController(kp, ki, kd);
    private PIDController rrPIDController = new PIDController(kp, ki, kd);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public MDrivetrain() {
        flMotor.setInverted(false);
        frMotor.setInverted(true);
        rlMotor.setInverted(false);
        rrMotor.setInverted(true);
        flMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        frMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rlMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rrMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    }

    @Override
    public void periodic() {
        pose = odometry.update(getHeading(), getWheelSpeeds());
        pidgey.getRawGyro(xyz_dps);
    }

    public Pose2d getPose() {
        return pose;
    }

    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public PIDController getXController() {
        return xController;
    }

    public PIDController getYController() {
        return yController;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        double flSpeed = flMotor.getSelectedSensorVelocity();
        double frSpeed = frMotor.getSelectedSensorVelocity();
        double rlSpeed = rlMotor.getSelectedSensorVelocity();
        double rrSpeed = rrMotor.getSelectedSensorVelocity();
        return new MecanumDriveWheelSpeeds(flSpeed, frSpeed, rlSpeed, rrSpeed);
    }

    // find a way to get desired rotation from trajectory.
    public Rotation2d getDesiredRotation() {
        return new Rotation2d();
    }

    public PIDController getFlPIDController() {
        return flPIDController;
    }

    public PIDController getFrPIDController() {
        return frPIDController;
    }

    public PIDController getRlPIDController() {
        return rlPIDController;
    }

    public PIDController getRrPIDController() {
        return rrPIDController;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public void setMotorOutput(MecanumDriveMotorVoltages mv) {
        // divided by 12 to get value between -1 and 1
        flMotor.set(ControlMode.PercentOutput, mv.frontLeftVoltage / 12);
        frMotor.set(ControlMode.PercentOutput, mv.frontRightVoltage / 12);
        rlMotor.set(ControlMode.PercentOutput, mv.rearLeftVoltage / 12);
        rrMotor.set(ControlMode.PercentOutput, mv.rearRightVoltage / 12);
    }

    private Rotation2d getHeading() {
        /**
         * Gyroscopes in frc return positive values as you turn clockwise; acording to
         * the unit circle this is backwards, thus we need to flip our gyro numbers.
         */
        return Rotation2d.fromDegrees(-pidgey.getFusedHeading());
    }

}
