package frc.robot.constants;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
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

  // World Constants

  // Misc
  public static final int kPowerDistributionPanelId = 0;
  public static final int kPigeonId = 50;

  // Autonomous Trajectories & File Paths
  public enum AutonomousTrajectory {
    DEFAULT(
        new Path[] {
          Filesystem.getDeployDirectory().toPath().resolve("paths/exitTarmac.wpilib.json")
        }),
    BALLTOHPTHENLAYUP(
        new Path[] {
          Filesystem.getDeployDirectory().toPath().resolve("paths/BallToHP.wpilib.json"),
          Filesystem.getDeployDirectory().toPath().resolve("paths/terminalToHub.wpilib.json")
        }),
    HOLONOMIC_TEST_PATH(
        new Path[] {
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("pathplanner/generatedJSON/HolonomicTestPath.wpilib.json")
        }),
    HOLONOMIC_TEST_PATH_2(
        new Path[] {
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("pathplanner/generatedJSON/HolonomicTestPath2.wpilib.json")
        });

    public Trajectory[] kTrajs;
    public Trajectory kFullTraj;

    private AutonomousTrajectory(Path[] trajPaths) {
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
