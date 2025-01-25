// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

public class FlywheelIOTalonFX implements FlywheelIO {
  //
  //  private final TalonFX leader = new TalonFX(0);
  //  private final TalonFX follower = new TalonFX(1);
  //
  //  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  //  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  //  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  //  private final StatusSignal<Current> leaderCurrent = leader.getStatorCurrent();
  //  private final StatusSignal<Current> followerCurrent = follower.getStatorCurrent();
  //
  //  public FlywheelIOTalonFX() {
  //    var config = new TalonFXConfiguration();
  //    config.CurrentLimits.StatorCurrentLimit = 30.0;
  //    config.CurrentLimits.StatorCurrentLimitEnable = true;
  //    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  //    leader.getConfigurator().apply(config);
  //    follower.getConfigurator().apply(config);
  //    follower.setControl(new Follower(leader.getDeviceID(), false));
  //
  //    BaseStatusSignal.setUpdateFrequencyForAll(
  //        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent,
  // followerCurrent);
  //    leader.optimizeBusUtilization();
  //    follower.optimizeBusUtilization();
  //  }
  //
  //  @Override
  //  public void updateInputs(FlywheelIOInputs inputs) {
  //    BaseStatusSignal.refreshAll(
  //        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
  //    inputs.shooterPositionRad =
  //        Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
  //    inputs.shooterVelocityRadPerSec =
  //        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
  //    inputs.shooterAppliedVolts = leaderAppliedVolts.getValueAsDouble();
  //    inputs.shooterCurrentAmps =
  //        new double[] {
  //          leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble(),
  //        };
  //  }
  //
  //  @Override
  //  public void setVoltage(double volts) {
  //    leader.setControl(new VoltageOut(volts));
  //  }
  //
  //  @Override
  //  public void setVelocity(double velocityRadPerSec, double ffVolts) {
  //    leader.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
  //  }
  //
  //  @Override
  //  public void stop() {
  //    leader.stopMotor();
  //  }
  //
  //  @Override
  //  public void configurePID(double kP, double kI, double kD) {
  //    var config = new Slot0Configs();
  //    config.kP = kP;
  //    config.kI = kI;
  //    config.kD = kD;
  //    leader.getConfigurator().apply(config);
  //  }
}