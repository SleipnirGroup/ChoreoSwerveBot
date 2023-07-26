package lib.choreolib;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryManager {
  private List<String> trajectories = new ArrayList<>();

  private final Gson gson = new Gson();

  private static TrajectoryManager instance;

  public static TrajectoryManager getInstance() {
    if (instance == null) {
      instance = new TrajectoryManager();
    }

    return instance;
  }

  public ChoreoTrajectory getTrajectory(String trajName) {
    var traj_dir = new File(Filesystem.getDeployDirectory(), "trajectories");
    var traj_file = new File(traj_dir, trajName);

    return loadFile(traj_file);
  }

  public void LoadTrajectories() {
    var traj_dir = new File(Filesystem.getDeployDirectory(), "trajectories");
    if (traj_dir.exists()) {
      File[] trajFiles= traj_dir.listFiles();
      for (File traj : trajFiles) {
        if (traj.isFile() && traj.getName().endsWith(".json")) {
          trajectories.add(traj.getName());
        }
      }
    }
  }

  private ChoreoTrajectory loadFile(File path) {
    try {
      var reader = new BufferedReader(new FileReader(path));
      var states = gson.fromJson(reader, ChoreoTrajectoryState[].class);

      return new ChoreoTrajectory(Arrays.asList(states));
    } catch (Exception ex) {
      DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
    }
    return null;
  }

  public List<String> getTrajectories() {
    return trajectories;
  }
}