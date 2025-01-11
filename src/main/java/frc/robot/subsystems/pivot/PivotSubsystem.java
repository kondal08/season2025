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
    IDLING(() -> PivotConstants.Hardware.RESTING_ANGLE),
    LEVEL_ONE(() -> PivotConstants.Hardware.RESTING_ANGLE + 0.1),
    LEVEL_TWO(() -> PivotConstants.Hardware.RESTING_ANGLE + 0.2),
    LEVEL_THREE(() -> PivotConstants.Hardware.RESTING_ANGLE + 0.3),
    LEVEL_FOUR(() -> PivotConstants.Hardware.MAX_ANGLE_RAD);

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
