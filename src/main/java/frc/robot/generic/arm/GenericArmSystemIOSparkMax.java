package frc.robot.generic.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class GenericArmSystemIOSparkMax implements GenericArmSystemIO {
  private final SparkMax[] motors;
  private final RelativeEncoder encoder;
  private double restingAngle;
  private final double reduction;
  private SparkBaseConfig config;
  private SparkClosedLoopController controller;

  public GenericArmSystemIOSparkMax(
      int[] id,
      boolean[] inverted,
      int currentLimitAmps,
      double restingAngle,
      boolean brake,
      double reduction,
      double kP,
      double kI,
      double kD) {
    this.reduction = reduction;
    this.restingAngle = restingAngle;
    motors = new SparkMax[id.length];
    config =
        new SparkMaxConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    config
        .closedLoop
        .pid(kP, kI, kD)
        .maxMotion
        .maxAcceleration(6000)
        .maxVelocity(6000)
        .allowedClosedLoopError(0.2);

    for (int i = 0; i < id.length; i++) {
      motors[i] = new SparkMax(id[i], SparkLowLevel.MotorType.kBrushless);

      if (i == 0)
        motors[i].configure(
            config.inverted(inverted[i]),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      else
        motors[i].configure(
            new SparkMaxConfig().apply(config.inverted(inverted[i])).follow(motors[0]),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }
    encoder = motors[0].getEncoder();
    controller = motors[0].getClosedLoopController();
  }

  public void updateInputs(GenericArmSystemIOInputs inputs) {
    if (motors != null) {
      inputs.degrees = Units.rotationsToDegrees(encoder.getPosition());
      inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
      inputs.appliedVoltage = motors[0].getAppliedOutput() * motors[0].getBusVoltage();
      inputs.supplyCurrentAmps = motors[0].getOutputCurrent();
      inputs.tempCelsius = motors[0].getMotorTemperature();
    }
  }

  @Override
  public void runToDegree(double degrees) {
    controller.setReference(degrees, SparkBase.ControlType.kMAXMotionPositionControl);
  }
}
