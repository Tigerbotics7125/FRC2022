package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

/**
 * A class to contain all things that remain constant and should be easily changed, without breaking
 * other code.
 *
 * <p>All measurements should be in SI units.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class Constants {

  // Shuffleboard tab names
  public static final String kPreGameTabName = "Pre-Game";
  public static final String kAutoTabName = "Auto";
  public static final String kTeleopTabName = "Teleop";

  // Differential Drivetrain
  public static final int kDDLeftDeviceId = 10;
  public static final int kDDRightDeviceId = 20;

  public static final double kDDWheelBaseWidth = Units.inchesToMeters(23);
  public static final double kDDWheelDiameter = Units.inchesToMeters(6);

  public static final double kDDGearRatio = 10.71;
  public static final double kDDRPMToMPSConversionFactor =
      1.0 / (kDDGearRatio * Math.PI * kDDWheelDiameter) / 60.0;
  public static final double kDDDistancePerPulse =
      1.0 / (42 / kDDGearRatio * (Math.PI * kDDWheelDiameter));

  // Mecanum Drivetrain
  public static final int kMDLeftFrontId = 1;
  public static final int kMDLeftRearId = 2;
  public static final int kMDRightFrontId = 3;
  public static final int kMDRightRearId = 4;

  public static final Translation2d kMDLeftFrontOffset = new Translation2d(1, 1); // TODO
  public static final Translation2d kMDLeftRearOffset = new Translation2d(-1, 1); // TODO
  public static final Translation2d kMDRightFrontOffset = new Translation2d(1, -1); // TODO
  public static final Translation2d kMDRightRearOffset = new Translation2d(-1, -1); // TODO

  public static final MotorType kMDMotorType = MotorType.kBrushless;

  public static final double kMDWheelBaseWidth = Units.inchesToMeters(23);
  public static final double kMDWheelDiameter = Units.inchesToMeters(6);

  public static final double kMDGearRatio = 10.71;
  public static final double kMDRPMtoMPSConversionFactor =
      1.0 / (kMDGearRatio * (Math.PI * kMDWheelDiameter));
  public static final double kMDDistancePerPulse =
      1.0 / (42 /* Hall Effects CPR */ / kMDGearRatio * (Math.PI * kMDWheelDiameter));

  // Misc
  public static final int kPowerDistributionPanelId = 0;
  public static final int kPigeonId = 50;

  // Auto Trajectories & File Paths
  public enum kAutoTrajs {
    DEFAULT(
        new Path[] {
          Filesystem.getDeployDirectory().toPath().resolve("paths/exitTarmac.wpilib.json")
        }),
    BALLTOHPTHENLAYUP(
        new Path[] {
          Filesystem.getDeployDirectory().toPath().resolve("paths/BallToHP.wpilib.json"),
          Filesystem.getDeployDirectory().toPath().resolve("paths/terminalToHub.wpilib.json")
        });

    public Trajectory[] kTrajs;
    public Trajectory kFullTraj;

    private kAutoTrajs(Path[] trajPaths) {
      Trajectory[] trajs = new Trajectory[trajPaths.length];
      Trajectory fullTraj = null;
      for (int i = 0; i < trajPaths.length; i++) {
        Path path = trajPaths[i];
        try {
          trajs[i] = (TrajectoryUtil.fromPathweaverJson(path));

          if (fullTraj == null) {
            fullTraj = trajs[i];
          } else {
            fullTraj = fullTraj.concatenate(trajs[i]);
          }
        } catch (IOException e) {
          DriverStation.reportError("Failed to open path: " + path.toString(), e.getStackTrace());
        }
      }
      this.kTrajs = trajs;
      this.kFullTraj = fullTraj;
    }
  }
}
