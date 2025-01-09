package frc.robot.subsystems.climber;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ClimberIOReal implements ClimberIO {
  private final SparkFlex leftClimber =
      new SparkFlex(ClimberConstants.LEFT_CLIMBER, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex rightClimber =
      new SparkFlex(ClimberConstants.RIGHT_CLIMBER, SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder lEncoder = leftClimber.getEncoder();
  private final RelativeEncoder rEncoder = rightClimber.getEncoder();
  private final SparkClosedLoopController lPid = leftClimber.getClosedLoopController();
  private final SparkClosedLoopController rPid = rightClimber.getClosedLoopController();
  private double climberPosition = 0.0;

  public ClimberIOReal() {
    var config =
        new SparkFlexConfig().inverted(false).voltageCompensation(12.0).smartCurrentLimit(30);

    leftClimber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftClimber.setCANTimeout(250);
    rightClimber.setCANTimeout(250);

    leftClimber.setInverted(false);
    rightClimber.setInverted(false);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberLeftPositionMeters = lEncoder.getPosition();
    inputs.climberRightPositionMeters = rEncoder.getPosition();
    inputs.climberCurrentAmps[0] = leftClimber.getOutputCurrent();
    inputs.climberCurrentAmps[1] = rightClimber.getOutputCurrent();
    inputs.climberLeftAppliedVolts = leftClimber.getBusVoltage();
    inputs.climberRightAppliedVolts = rightClimber.getBusVoltage();

    inputs.climberSetpointPosition = climberPosition;
  }

  @Override
  public void setPosition(double climberPositionRad) {
    lPid.setReference(climberPositionRad, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rPid.setReference(climberPositionRad, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    climberPosition = climberPositionRad;
  }

  @Override
  public void setVoltage(double volts) {
    leftClimber.setVoltage(volts);
    rightClimber.setVoltage(volts);
  }

  @Override
  @Deprecated
  public void setLeftVoltage(double volts) {
    leftClimber.setVoltage(volts);
  }

  @Override
  @Deprecated
  public void setRightVoltage(double volts) {
    rightClimber.setVoltage(volts);
  }

  @Override
  @Deprecated
  public void setHoming(boolean homingBool) {}

  @Override
  public void resetEncoder(final double position) {
    lEncoder.setPosition(0);
    rEncoder.setPosition(0);
  }

  @Override
  public boolean isCurrentLimited() {
    return leftClimber.getOutputCurrent() > 50;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // Left PID Values
    lPid.setP(kP, 0);
    lPid.setI(kI, 0);
    lPid.setD(kD, 0);
    lPid.setFF(0, 0);
    // Right PID Values
    rPid.setP(kP, 0);
    rPid.setI(kI, 0);
    rPid.setD(kD, 0);
    rPid.setFF(0, 0);
  }
}
