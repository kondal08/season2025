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
public class ElevatorSubsystem extends GenericPositionElevatorSystem<ElevatorSubsystem.ElevatorGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ElevatorGoal implements GenericPositionElevatorSystem.ExtensionGoal {
    IDLING(() -> 0.0), // Climber is off
    DEEP_CLIMB(() -> 0.5), // Deep climb level
    SHALLOW_CLIMB(() -> 10); // Deep climb level

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
