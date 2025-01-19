package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static java.lang.Math.PI;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generic.elevators.GenericPositionElevatorSystem;
import frc.robot.util.LoggedTunableNumber;
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
    LEVEL_ONE(() -> 0.46),
    LEVEL_TWO(() -> 0.81),
    LEVEL_THREE(() -> 1.21),
    LEVEL_FOUR(() -> 1.83),
    TESTING(new LoggedTunableNumber("Elevator/Test", 0.0));

    private final DoubleSupplier heightSupplier;

    @Override
    public Distance getHeightSupplier() {
      return Meters.of(heightSupplier.getAsDouble() / (2 * PI * ElevatorConstants.radius));
    }
  }

  private ElevatorGoal goal = ElevatorGoal.IDLING;

  public ElevatorSubsystem(String name, ElevatorIO io) {
    super(name, io);
  }
}
