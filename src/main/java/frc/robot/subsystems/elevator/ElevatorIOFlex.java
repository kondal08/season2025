package frc.robot.subsystems.elevator;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkFlex;

public class ElevatorIOFlex extends GenericElevatorSystemIOSparkFlex implements ElevatorIO {
  public ElevatorIOFlex() {
    super(
        new int[] {ElevatorConstants.RIGHT_ELEVATOR, ElevatorConstants.LEFT_ELEVATOR},
        new boolean[] {ElevatorConstants.RIGHT_INVERTED, ElevatorConstants.LEFT_INVERTED},
        40,
        0.0,
        true,
        1.0,
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
            ElevatorConstants.DIOPort);
  }
}
