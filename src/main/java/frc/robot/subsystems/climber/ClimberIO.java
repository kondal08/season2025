package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberLeftPositionMeters = 0.0;
    public double climberRightPositionMeters = 0.0;
    public double climberLeftAppliedVolts = 0.0;
    public double climberRightAppliedVolts = 0.0;
    public double[] climberCurrentAmps =
        new double[] {}; // Log motors individually, useful for failure analysis
    public double climberSetpointPosition = 0.0;

    public boolean openLoopStatus = true;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ClimberIOInputs inputs);

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts);
}
