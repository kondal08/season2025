package frc.robot.subsystems.feeder;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

import com.revrobotics.spark.SparkMax;

public class FeederIOReal implements FeederIO {
  private final SparkMax feeder = new SparkMax(FeederConstants.FEEDER_ID, kBrushless);
  // private final DigitalInput beamBrake = new DigitalInput(1);

  @Override
  public void updateInputs(FeederIO.FeederIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {
    feeder.setVoltage(volts);
  }

  public boolean hasNote() {
    // return beamBrake.get();
    return false;
  }
}
