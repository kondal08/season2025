// change between sim and sparkmax files

package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class ClimberIOSim extends GenericRollerSystemIOSim implements ClimberIO {
    public ClimberIOSim(DCMotor motorModel, double reduction, double moi) {
        super(motorModel, reduction, moi);
    }
}
