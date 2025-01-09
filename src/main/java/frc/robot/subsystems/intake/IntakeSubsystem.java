package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.generic.rollers.GenericVoltageRollerSystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class IntakeSubsystem extends GenericVoltageRollerSystem<IntakeSubsystem.IntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IntakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    INTAKING(() -> 12.0), // Maximum forward voltage
    EJECTING(() -> -12.0); // Maximum reverse voltage

    private final DoubleSupplier voltageSupplier;
  }

  private IntakeGoal goal = IntakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public IntakeSubsystem(String name, IntakeIO io) {
    super(name, io);
  }

  public boolean hasNote() {
    return goal == IntakeGoal.INTAKING
        && stateTimer.hasElapsed(0.25)
        && currentDebouncer.calculate(inputs.torqueCurrentAmps > 45.0);
  }
}
