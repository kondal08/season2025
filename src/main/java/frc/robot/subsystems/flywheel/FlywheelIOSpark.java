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

import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkBase.PersistMode;
import static com.revrobotics.spark.SparkBase.ResetMode;
import static frc.robot.subsystems.flywheel.FlywheelConstants.GEAR_RATIO;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

// NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
// "CANSparkFlex"
public class FlywheelIOSpark implements FlywheelIO {

  private final SparkFlex leader =
      new SparkFlex(FlywheelConstants.TOP_FLYWHEEL, MotorType.kBrushless);
  private final SparkFlex follower =
      new SparkFlex(FlywheelConstants.BOTTOM_FLYWHEEL, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController pid = leader.getClosedLoopController();

  public FlywheelIOSpark() {
    var leaderConfig =
        new SparkFlexConfig()
            .voltageCompensation(12.0)
            .smartCurrentLimit(30)
            .idleMode(SparkBaseConfig.IdleMode.kCoast);
    var followerConfig = new SparkFlexConfig().apply(leaderConfig).follow(leader, true);

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leader.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.shooterPositionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.shooterVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.shooterAppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.shooterCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new ClosedLoopConfig().pid(kP, kI, kD);
    leader.configure(
        new SparkFlexConfig().apply(config),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
