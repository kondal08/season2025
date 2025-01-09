package frc.robot.subsystems.pivot;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.subsystems.pivot.PivotConstants.Hardware.ABSOLUTE_ENCODER_OFFSET;
import static java.lang.Math.PI;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOReal implements PivotIO {
  private final SparkFlex leader = new SparkFlex(Software.LEFT_PIVOT_ID, kBrushless),
      follower = new SparkFlex(Software.RIGHT_PIVOT_ID, kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final DutyCycleEncoder absEncoder =
      new DutyCycleEncoder(PivotConstants.Software.ENCODER_PORT);

  public PivotIOReal() {
    var leaderConfig =
        new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40);
    var followerConfig =
        new SparkFlexConfig().apply(leaderConfig).follow(leader.getDeviceId(), true);

    leader.configure(
        leaderConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    follower.configure(
        followerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(PivotIO.PivotIOInputs inputs) {
    inputs.appliedVolts = leader.getBusVoltage();
    inputs.currentAmps = leader.getOutputCurrent();
    inputs.positionRad = leader.getEncoder().getPosition();
    inputs.absolutePositionRad = (absEncoder.get() - ABSOLUTE_ENCODER_OFFSET) * 2 * PI;
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }
}
