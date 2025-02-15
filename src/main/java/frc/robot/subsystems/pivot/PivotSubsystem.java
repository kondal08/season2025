package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import frc.robot.generic.arm.GenericPositionArmSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class PivotSubsystem extends GenericPositionArmSystem<PivotSubsystem.PivotGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum PivotGoal implements GenericPositionArmSystem.PivotGoal {
    IDLING(() -> 45),
    SOURCE(() -> 100),
    LEVEL_ONE(() -> 60),
    LEVEL_TWO(() -> 60),
    LEVEL_THREE(() -> 60),
    LEVEL_FOUR(() -> 50),
    TESTING(new LoggedTunableNumber("Pivot/TESTING", 50.0));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngle() {
      return () -> Units.degreesToRotations(angleSupplier.getAsDouble());
    }
  }

  private PivotGoal goal = PivotGoal.IDLING;

  public PivotSubsystem(String name, PivotIO io) {
    super(name, io);
  }
}
