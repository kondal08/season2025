package frc.robot.subsystems;

import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Superstructure extends SubsystemBase {
  /**
   * For use during autonomous and the Superstructure's periodic. The action (non-idle) entries end
   * in "ing" intentionally – if the robot is not in the state of actively transitioning between
   * states, it's idling.
   */
  private Supplier<Pose2d> drivePoseSupplier;

  private final PivotSubsystem pivot =
      Config.Subsystems.ELEVATOR_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new PivotSubsystem("Pivot", new PivotIOReal())
              : new PivotSubsystem("Pivot Sim", new PivotIOSim(2, 0.0)))
          : null;
  private final FeederSubsystem feeder =
      Config.Subsystems.FEEDER_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new FeederSubsystem("Feeder", new FeederIOReal())
              : new FeederSubsystem(
                  "Feeder Sim", new FeederIOSim(new DCMotor(1, 1, 1, 1, 1, 1), 0, 0)))
          : null;

  private final ClimberSubsystem climber =
      Config.Subsystems.ELEVATOR_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new ClimberSubsystem("Climber", new ClimberIOReal())
              : new ClimberSubsystem("Climber Sim", new ClimberIOSim(2, 0.0)))
          : null;

  private final ElevatorSubsystem elevator =
      Config.Subsystems.ELEVATOR_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new ElevatorSubsystem("Climber", new ElevatorIOReal())
              : new ElevatorSubsystem("Climber Sim", new ElevatorIOSim(2, 0.0)))
          : null;

  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  public Superstructure(Supplier<Pose2d> drivePoseSupplier) {
    this.drivePoseSupplier = drivePoseSupplier;
  }

  public static enum SuperStates {
    IDLING
  }

  /**
   * For use during teleop to modify the current SuperState. Each one requests a state in {@link
   * SuperStates}. REQ_NONE is the absence of
   */
  public static enum StateRequests {
    REQ_NONE,
    REQ_IDLE,
    REQ_INTAKE,
    REQ_SHOOT
  }

  private SuperStates currentState = IDLING;

  public Command setSuperStateCmd(SuperStates stateRequest) {
    return Commands.run(() -> currentState = stateRequest);
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLING -> {
        if (feeder != null) feeder.setGoal(FeederSubsystem.FeederGoal.IDLING);
        // if (flywheel != null)
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.IDLING);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.IDLING);
        if (elevator != null) elevator.setGoal(ElevatorSubsystem.ElevatorGoal.IDLING);
        if (leds != null)
          leds.setRunAlongCmd(
              () -> AllianceFlipUtil.shouldFlip() ? Color.kRed : Color.kBlue,
              () -> Color.kBlack,
              5,
              1);
      }
    }
  }

  public Command setLEDBlinkingCmd(Color onColor, Color offColor, double frequency) {
    if (leds != null) return leds.setBlinkingCmd(onColor, offColor, frequency);
    else return null;
  }

  public void registerSuperstructureCharacterization(
      Supplier<LoggedDashboardChooser<Command>> autoChooser) {
  }
}
