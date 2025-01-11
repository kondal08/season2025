package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generic.arm.GenericPositionArmSystem;
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
    IDLING(() -> 0.0),
    LEVEL_ONE(() -> 0.1),
    LEVEL_TWO(() -> 0.2),
    LEVEL_THREE(() -> 0.3),
    LEVEL_FOUR(() -> 0.4);

    private final DoubleSupplier heightSupplier;

    @Override
    public Distance getHeightSupplier() {
      return Meters.of(heightSupplier.getAsDouble());
    }
  }

  private PivotGoal goal = PivotGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public PivotSubsystem(String name, PivotIO io) {
    super(name, io);
  }
}
