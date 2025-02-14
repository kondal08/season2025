package frc.robot.subsystems.pivot;

import frc.robot.generic.arm.GenericArmSystemIOSparkMax;
import frc.robot.subsystems.climber.ClimberConstants;

public class PivotIOMax extends GenericArmSystemIOSparkMax implements PivotIO {

  public PivotIOMax() {
    super(
        new int[] {PivotConstants.PIVOT_ID},
        new boolean[] {PivotConstants.INVERTED},
        40,
        0.0,
        true,
        1.0,
        ClimberConstants.kP,
        ClimberConstants.kI,
        ClimberConstants.kD);
  }
}
