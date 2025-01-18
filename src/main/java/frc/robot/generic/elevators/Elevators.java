package frc.robot.generic.elevators;

import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import lombok.Getter;

public class Elevators extends SubsystemBase {
  @Getter
  private final ClimberSubsystem climber =
      Config.Subsystems.ELEVATOR_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new ClimberSubsystem("Climber", new ClimberIOReal())
              : new ClimberSubsystem("Climber Sim", new ClimberIOSim(2, 0.0)))
          : null;

  @Getter
  private final ElevatorSubsystem elevator =
      Config.Subsystems.ELEVATOR_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new ElevatorSubsystem("Climber", new ElevatorIOReal())
              : new ElevatorSubsystem("Climber Sim", new ElevatorIOSim(2, 0.0)))
          : null;

  @Override
  public void periodic() {
    elevator.periodic();
    climber.periodic();
  }
}
