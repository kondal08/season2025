package frc.robot.generic.arm;

import static frc.robot.Config.Subsystems.PIVOT_ENABLED;
import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import lombok.Getter;

public class Arms extends SubsystemBase {
  @Getter
  private final PivotSubsystem pivot =
      Config.Subsystems.PIVOT_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new PivotSubsystem("Pivot", new PivotIOReal())
              : new PivotSubsystem("Pivot Sim", new PivotIOSim(2, 0.0)))
          : null;

  @Override
  public void periodic() {
    if (PIVOT_ENABLED) pivot.periodic();
  }
}
