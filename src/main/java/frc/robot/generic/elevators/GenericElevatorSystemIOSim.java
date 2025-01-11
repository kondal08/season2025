package frc.robot.generic.elevators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.generic.rollers.GenericRollerSystemIO;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;

public class GenericElevatorSystemIOSim implements GenericElevatorSystemIO {
    private final DCMotorSim sim;
    private double appliedVoltage = 0.0;

    public GenericElevatorSystemIOSim(DCMotor motorModel, double reduction, double moi) {
        sim = new DCMotorSim(createDCMotorSystem(motorModel, reduction, moi), motorModel);
    }


    public void updateInputs(GenericElevatorSystemIO.GenericElevatorSystemIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            runVolts(appliedVoltage);
        }

        sim.update(0.02);
        inputs.positionMeters = sim.getAngularPositionRad();
        inputs.velocityMetersPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    }


    public void runVolts(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }
}
