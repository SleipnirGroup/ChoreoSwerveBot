package lib.choreolib;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class ChoreoTrajectory {
  private final List<ChoreoTrajectoryState> states;

  public ChoreoTrajectory() {
    states = List.of();
  }

  private ChoreoTrajectoryState sampleInternal(double timestamp) {
    if (timestamp < states.get(0).getTimestamp()) {
      return states.get(0);
    }
    if (timestamp > getTotalTime()) {
      return states.get(states.size() - 1);
    }

    int low = 0;
    int high = states.size() - 1;

    while (low != high) {
      int mid = (low + high) / 2;
      if (states.get(mid).getTimestamp() < timestamp) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }

    if (low == 0) {
      return states.get(low);
    }

    var behindState = states.get(low - 1);
    var aheadState = states.get(low);

    if ((aheadState.getTimestamp() - behindState.getTimestamp()) < 1e-6) {
      return aheadState;
    }

    return behindState.interpolate(aheadState, timestamp);
  }

  public ChoreoTrajectoryState sample(double timestamp) {
    return sample(timestamp, false);
  }

  public ChoreoTrajectoryState sample(double timestamp, boolean mirrorForRedAlliance) {
    var state = sampleInternal(timestamp);
    return mirrorForRedAlliance ? state.flipped() : state;
  }

  public Pose2d getInitialPose() {
    return states.get(0).getPose();
  }

  public Pose2d getFinalPose() {
    return states.get(states.size() - 1).getPose();
  }

  public double getTotalTime() {
    return states.get(states.size() - 1).getTimestamp();
  }

  public Pose2d[] getPoses() {
    return states.stream().map((state)->state.getPose()).toArray(Pose2d[]::new);
  }

  public ChoreoTrajectory mirrorred() {
    var flippedStates = new ArrayList<ChoreoTrajectoryState>();
    for (var state : states) {
      flippedStates.add(state.flipped());
      System.out.println(state.flipped().getPose());
    }
    return new ChoreoTrajectory(flippedStates);
  }
}