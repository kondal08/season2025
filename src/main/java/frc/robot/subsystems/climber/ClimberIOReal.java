package frc.robot.subsystems.climber;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class ClimberIOReal extends GenericRollerSystemIOSparkMax implements ClimberIO {
    public ClimberIOReal(int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
        super(id, currentLimitAmps, invert, brake, reduction);
    }
}
