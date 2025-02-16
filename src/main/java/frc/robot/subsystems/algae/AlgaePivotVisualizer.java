package frc.robot.subsystems.algae;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class AlgaePivotVisualizer {

  private final String name;

  public AlgaePivotVisualizer(String name) {
    this.name = name;
  }

  public void update(double angleRads) {
    Pose3d pivot = new Pose3d(-0.25, 0, 0.155, new Rotation3d(0, -angleRads, 0));
    Logger.recordOutput("Pivot/" + name, pivot);
  }
}
