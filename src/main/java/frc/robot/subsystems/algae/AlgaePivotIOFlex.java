package frc.robot.subsystems.algae;

import static frc.robot.subsystems.algae.AlgaePivotConstants.FORWARD_LIMIT;
import static frc.robot.subsystems.algae.AlgaePivotConstants.REVERSE_LIMIT;

import frc.robot.generic.arm.GenericArmSystemIOSparkFlex;

public class AlgaePivotIOFlex extends GenericArmSystemIOSparkFlex implements AlgaePivotIO {

  public AlgaePivotIOFlex() {
    super(
        new int[] {AlgaePivotConstants.PIVOT_ID},
        new boolean[] {AlgaePivotConstants.INVERTED},
        40,
        true,
        FORWARD_LIMIT,
        REVERSE_LIMIT,
        AlgaePivotConstants.kP);
  }
}
