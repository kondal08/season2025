package frc.robot.subsystems;

import static frc.robot.Config.Subsystems.ALGAE_INTAKE_ENABLED;
import static frc.robot.Config.Subsystems.CLIMBER_ENABLED;
import static frc.robot.Config.Subsystems.CORAL_INTAKE_ENABLED;
import static frc.robot.Config.Subsystems.ELEVATOR_ENABLED;
import static frc.robot.Config.Subsystems.LEDS_ENABLED;
import static frc.robot.Config.Subsystems.PIVOT_ENABLED;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.generic.arm.Arms;
import frc.robot.generic.elevators.Elevators;
import frc.robot.generic.rollers.Rollers;
import frc.robot.subsystems.algaeintake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralintake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Superstructure extends SubsystemBase {
  /**
   * For use during autonomous and the Superstructure's periodic. The action (non-idle) entries end
   * in "ing" intentionally – if the robot is not in the state of actively transitioning between
   * states, it's idling.
   */
  private Supplier<Pose2d> drivePoseSupplier;

  private final Arms arms = new Arms();
  private final Elevators elevators = new Elevators();
  private final Rollers rollers = new Rollers();
  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  public Superstructure(Supplier<Pose2d> drivePoseSupplier) {
    this.drivePoseSupplier = drivePoseSupplier;
    if (LEDS_ENABLED)
      leds.setDefaultCommand(
          leds.ledCommand(
              DriverStation::isEnabled,
              DriverStation::isFMSAttached,
              () -> (DriverStation.getMatchTime() <= 30),
              () -> true,
              () -> false,
              ALGAE_INTAKE_ENABLED ? rollers.getAlgaeIntake().hasAlgae() : () -> false,
              CORAL_INTAKE_ENABLED ? rollers.getCoralIntake().hasCoral() : () -> false));
  }

  public static enum SuperStates {
    IDLING,
    TESTING,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE,
    LEVEL_FOUR,
    OUTAKE,
    INTAKE,
    STOP_INTAKE,
    SOURCE,
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

  private boolean wantsCoral = true;

  public Command setCoralModeCmd(boolean wantCoralMode) {
    return Commands.run(() -> wantsCoral = wantCoralMode);
  }

  public Command switchCoralMode() {
    return Commands.run(() -> wantsCoral = !wantsCoral);
  }

  private SuperStates currentState = IDLING;

  public Command setSuperStateCmd(SuperStates stateRequest) {
    return Commands.run(() -> currentState = stateRequest);
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLING -> {
        if (CORAL_INTAKE_ENABLED)
          rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.IDLING);
        if (ALGAE_INTAKE_ENABLED)
          rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.IDLING);
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.IDLING);
        if (CLIMBER_ENABLED) elevators.getClimber().setGoal(ClimberSubsystem.ClimberGoal.IDLING);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.IDLING);
      }
      case TESTING -> {
        if (CORAL_INTAKE_ENABLED)
          rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.FORWARD);
        if (ALGAE_INTAKE_ENABLED)
          rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.FORWARD);
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.TESTING);
        if (CLIMBER_ENABLED) elevators.getClimber().setGoal(ClimberSubsystem.ClimberGoal.TESTING);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.TESTING);
      }
      case LEVEL_ONE -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_ONE);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_ONE);
      }
      case LEVEL_TWO -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_TWO);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_TWO);
      }
      case LEVEL_THREE -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_THREE);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_THREE);
      }
      case LEVEL_FOUR -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_FOUR);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_FOUR);
      }
      case INTAKE -> {
        if (wantsCoral) {
          if (CORAL_INTAKE_ENABLED)
            rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.FORWARD);
        } else {
          if (ALGAE_INTAKE_ENABLED)
            rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.FORWARD);
        }
      }
      case OUTAKE -> {
        if (wantsCoral) {
          if (CORAL_INTAKE_ENABLED)
            rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.REVERSE);
        } else {
          if (ALGAE_INTAKE_ENABLED)
            rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.REVERSE);
        }
      }
      case STOP_INTAKE -> {
        if (CORAL_INTAKE_ENABLED)
          rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.IDLING);
      }
      case SOURCE -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.SOURCE);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.SOURCE);
      }
    }
  }

  public Trigger coralMode() {
    return new Trigger(() -> wantsCoral);
  }

  public Trigger algaeScoringMode() {
    // return new Trigger(() -> !wantsCoral && rollers.getAlgaeIntake().hasAlgae().getAsBoolean());
    return new Trigger(() -> true);
  }

  public Trigger algaeIntakeMode() {
    // return new Trigger(() -> !wantsCoral && !rollers.getAlgaeIntake().hasAlgae().getAsBoolean());
    return new Trigger(() -> false);
  }

  public void registerSuperstructureCharacterization(
      Supplier<LoggedDashboardChooser<Command>> autoChooser) {}
}
