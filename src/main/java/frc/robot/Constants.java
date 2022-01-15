package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

public class Constants {

  // drive
  public static final double kWheelBaseWidthMeters = Units.inchesToMeters(23);
  public static final double kWheelDiameter = Units.inchesToMeters(6);
  public static final int kLeftDeviceId = 1;
  public static final int kRightDeviceId = 2;

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

    private kAutoTrajs(Path[] trajPaths) {
      Trajectory[] trajs = new Trajectory[trajPaths.length];
      for (int i = 0; i < trajPaths.length; i++) {
        Path path = trajPaths[i];
        try {
          trajs[i] = (TrajectoryUtil.fromPathweaverJson(path));
        } catch (IOException e) {
          DriverStation.reportError("Failed to open path: " + path.toString(), e.getStackTrace());
        }
      }
      this.kTrajs = trajs;
    }
  }
}
