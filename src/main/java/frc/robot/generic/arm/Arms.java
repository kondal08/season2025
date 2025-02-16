package frc.robot.generic.arm;

import static frc.robot.Config.Subsystems.*;
import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.algae.*;
import frc.robot.subsystems.coral.CoralPivotConstants;
import frc.robot.subsystems.coral.CoralPivotIOFlex;
import frc.robot.subsystems.coral.CoralPivotIOMax;
import frc.robot.subsystems.coral.CoralPivotIOSim;
import frc.robot.subsystems.coral.CoralPivotSubsystem;
import lombok.Getter;

public class Arms extends SubsystemBase {
  @Getter
  private final CoralPivotSubsystem coralPivot =
      CORAL_PIVOT_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? CoralPivotConstants.IS_FLEX
                  ? new CoralPivotSubsystem("Pivot", new CoralPivotIOFlex())
                  : new CoralPivotSubsystem("Pivot", new CoralPivotIOMax())
              : new CoralPivotSubsystem("Pivot Sim", new CoralPivotIOSim(2, 0.0)))
          : null;

  @Getter
  private final AlgaePivotSubsystem algaePivot =
      Config.Subsystems.ALGAE_PIVOT_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? AlgaePivotConstants.IS_FLEX
                  ? new AlgaePivotSubsystem("Pivot", new AlgaePivotIOFlex())
                  : new AlgaePivotSubsystem("Pivot", new AlgaePivotIOMax())
              : new AlgaePivotSubsystem("Pivot Sim", new AlgaePivotIOSim(2, 0.0)))
          : null;

  @Override
  public void periodic() {
    if (CORAL_PIVOT_ENABLED) coralPivot.periodic();
    if (ALGAE_PIVOT_ENABLED) algaePivot.periodic();
  }
}
