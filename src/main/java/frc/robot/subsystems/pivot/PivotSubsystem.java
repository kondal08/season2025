package frc.robot.subsystems.pivot;

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
    IDLING(new LoggedTunableNumber("Pivot/idle", 0.0)),
    SOURCE(() -> 0.38),
    LEVEL_ONE(() -> 0.255),
    LEVEL_TWO(() -> 0.17),
    LEVEL_THREE(() -> 0.17),
    LEVEL_FOUR(() -> 0.23),
    TESTING(new LoggedTunableNumber("Pivot/TESTING", 0.0));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngle() {
      return () -> angleSupplier.getAsDouble();
    }
  }

  private PivotGoal goal = PivotGoal.IDLING;

  public PivotSubsystem(String name, PivotIO io) {
    super(name, io);
  }
}
