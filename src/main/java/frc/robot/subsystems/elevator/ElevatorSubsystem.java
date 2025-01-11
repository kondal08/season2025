package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generic.elevators.GenericPositionElevatorSystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ElevatorSubsystem
    extends GenericPositionElevatorSystem<ElevatorSubsystem.ElevatorGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ElevatorGoal implements GenericPositionElevatorSystem.ExtensionGoal {
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

  private ElevatorGoal goal = ElevatorGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public ElevatorSubsystem(String name, ElevatorIO io) {
    super(name, io);
  }
}
