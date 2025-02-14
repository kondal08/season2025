package frc.robot.subsystems.elevator;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkFlex;
import frc.robot.subsystems.climber.ClimberConstants;

public class ElevatorIOFlex extends GenericElevatorSystemIOSparkFlex implements ElevatorIO {
  public ElevatorIOFlex() {
    super(
        new int[] {ElevatorConstants.RIGHT_ELEVATOR, ElevatorConstants.LEFT_ELEVATOR},
        new boolean[] {ElevatorConstants.RIGHT_INVERTED, ElevatorConstants.LEFT_INVERTED},
        40,
        0.0,
        true,
        1.0,
        ClimberConstants.kP,
        ClimberConstants.kI,
        ClimberConstants.kD);
  }
}
