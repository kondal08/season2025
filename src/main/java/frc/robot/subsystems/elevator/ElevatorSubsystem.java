package frc.robot.subsystems.elevator;

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
    CORAL_STATION(() -> 0.25),
    LEVEL_ONE(() -> 0.1),
    LEVEL_TWO(() -> 0.2),
    LEVEL_THREE(() -> 0.3),
    LEVEL_FOUR(() -> 0.4);

    private final DoubleSupplier height;
  }

  private ElevatorGoal goal = ElevatorGoal.IDLING;

  public ElevatorSubsystem(String name, ElevatorIO io) {
    super(name, io);
  }
}
