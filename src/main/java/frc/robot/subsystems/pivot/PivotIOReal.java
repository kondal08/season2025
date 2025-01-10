package frc.robot.subsystems.pivot;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class PivotIOReal extends GenericRollerSystemIOSparkMax implements PivotIO {
    public PivotIOReal(int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
        super(id, currentLimitAmps, invert, brake, reduction);
    }
}
