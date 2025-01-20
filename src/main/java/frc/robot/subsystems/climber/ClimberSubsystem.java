package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.generic.elevators.GenericPositionElevatorSystem;
import frc.robot.util.LoggedTunableNumber;
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
    SHALLOW_CLIMB(() -> 10),
    TESTING(new LoggedTunableNumber("Climber/Test", 0.0)); // Deep climb level

    private final DoubleSupplier height;

    @Override
    public DoubleSupplier getHeight() {
      return () -> height.getAsDouble() / (2 * Math.PI * ClimberConstants.PULLEY_RADIUS);
    }
  }

  private ClimberGoal goal = ClimberGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public ClimberSubsystem(String name, ClimberIO io) {
    super(name, io);
  }
}
