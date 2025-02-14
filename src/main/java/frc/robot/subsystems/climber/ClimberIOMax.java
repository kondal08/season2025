package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkMax;

public class ClimberIOMax extends GenericElevatorSystemIOSparkMax implements ClimberIO {
  public ClimberIOMax() {
    super(
        new int[] {ClimberConstants.RIGHT_CLIMBER, ClimberConstants.LEFT_CLIMBER},
        new boolean[] {ClimberConstants.RIGHT_INVERTED, ClimberConstants.LEFT_INVERTED},
        40,
        0.0,
        false,
        1.0,
        ClimberConstants.kP,
        ClimberConstants.kI,
        ClimberConstants.kD);
  }
}
