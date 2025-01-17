package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generic.elevators.GenericPositionElevatorSystem;
import frc.robot.generic.elevators.GenericPositionElevatorSystem.ExtensionGoal;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ClimberSubsystem extends GenericPositionElevatorSystem<ClimberSubsystem.ClimberGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ClimberGoal implements ExtensionGoal {
    IDLING(() -> 0.0), // Climber is off
    DEEP_CLIMB(() -> 0.5), // Deep climb level
    SHALLOW_CLIMB(() -> 10); // Deep climb level

    private final DoubleSupplier heightSupplier;

    @Override
    public Distance getHeightSupplier() {
      return Meters.of(heightSupplier.getAsDouble());
    }
  }

  private ClimberGoal goal = ClimberGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public ClimberSubsystem(String name, ClimberIO io) {
    super(name, io);
  }
}
