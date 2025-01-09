package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.Hardware.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends ProfiledPIDSubsystem {

  public static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/FeedbackGains/kP/", PivotConstants.Software.gains.kP());
  public static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Pivot/FeedbackGains/kI/", PivotConstants.Software.gains.kI());
  public static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Pivot/FeedbackGains/kD/", PivotConstants.Software.gains.kD());
  public static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Pivot/FeedforwardGains/kS/", PivotConstants.Software.gains.kS());
  public static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Pivot/FeedforwardGains/kV/", PivotConstants.Software.gains.kV());
  public static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Pivot/FeedforwardGains/kG/", PivotConstants.Software.gains.kG());
  public static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Pivot/ProfileConstraints/MaxVelocity/",
          PivotConstants.Software.profileConstraints.maxVelocity);
  public static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Pivot/ProfileConstraints/MaxAcceleration/",
          PivotConstants.Software.profileConstraints.maxAcceleration);

  private final PivotIO io;
  private final PivotVisualizer actualVisualizer, targetVisualizer, setpointVisualizer;

  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private ArmFeedforward ffModel;

  public final Trigger isReady;

  public PivotSubsystem(PivotIO io) {
    super(
        new ProfiledPIDController(
            kP.get(),
            kI.get(),
            kD.get(),
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())));
    this.io = io;
    ffModel = new ArmFeedforward(kS.get(), kG.get(), kV.get());

    m_controller.setTolerance(
        PivotConstants.Software.POSITION_TOLERANCE, PivotConstants.Software.VELOCITY_TOLERANCE);

    setGoal(RESTING_ANGLE);
    enable();

    actualVisualizer = new PivotVisualizer("ActualVisualizer");
    targetVisualizer = new PivotVisualizer("TargetVisualizer");
    setpointVisualizer = new PivotVisualizer("SetpointVisualizer");
    isReady = new Trigger(this.getController()::atGoal).debounce(0.3);
  }

  private void setPosition(DoubleSupplier angleRad) {
    if (angleRad.getAsDouble() < MAX_ANGLE_RAD && angleRad.getAsDouble() > MIN_ANGLE_RAD)
      setGoal(angleRad.getAsDouble());
  }

  public Command setHome() {
    return Commands.runOnce(() -> setPosition(() -> RESTING_ANGLE));
  }

  public Command setSubwoofer() {
    return Commands.runOnce(() -> setPosition(() -> 1));
  }

  public Command setAiming() {
    return Commands.runOnce(
        () -> {
          // LUT aiming will go IN HERE, no need to make a new command!!
          // LUT should be created in GlobalConstants? or a new class?
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> io.setVoltage(voltage));
  }

  @Override
  protected void useOutput(double v, TrapezoidProfile.State state) {
    var feedforward = ffModel.calculate(state.position, state.velocity);

    io.setVoltage(v + feedforward);
  }

  @Override
  protected double getMeasurement() {
    return inputs.absolutePositionRad;
  }

  @Override
  public void periodic() {
    super.periodic();

    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    getController().setPID(kP.get(), kI.get(), kD.get());

    Logger.recordOutput("Pivot/position", getMeasurement());
    Logger.recordOutput("Pivot/setpoint", getController().getGoal().position);

    actualVisualizer.update(getMeasurement());
    targetVisualizer.update(getController().getGoal().position);
    setpointVisualizer.update(getController().getSetpoint().position);
  }
}
