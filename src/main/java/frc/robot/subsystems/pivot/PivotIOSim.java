// change between sim and sparkmax files

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class PivotIOSim extends GenericRollerSystemIOSim implements PivotIO {
    public PivotIOSim(DCMotor motorModel, double reduction, double moi) {
        super(motorModel, reduction, moi);
    }
}
