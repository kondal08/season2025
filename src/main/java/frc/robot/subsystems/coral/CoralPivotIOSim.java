// change between sim and sparkmax files

package frc.robot.subsystems.coral;

import frc.robot.generic.arm.GenericArmSystemIOSim;

public class CoralPivotIOSim extends GenericArmSystemIOSim implements CoralPivotIO {
  public CoralPivotIOSim(int numMotors, double startingAngle) {
    super(numMotors, startingAngle);
  }
}
