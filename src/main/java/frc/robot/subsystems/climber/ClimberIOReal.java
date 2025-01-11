package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkMax;

public class ClimberIOReal extends GenericElevatorSystemIOSparkMax implements ClimberIO {
    public ClimberIOReal() {
        super(
                new int[]{ClimberConstants.LEFT_CLIMBER,
                        ClimberConstants.RIGHT_CLIMBER},
                40,
                0.0,
                false,
                true,
                0.0,
                ClimberConstants.kP,
                ClimberConstants.kI,
                ClimberConstants.kD);
    }
}
