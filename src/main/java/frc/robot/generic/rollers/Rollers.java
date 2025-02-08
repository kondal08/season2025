package frc.robot.generic.rollers;

import static frc.robot.Config.Subsystems.INTAKE_ALGAE_ENABLED;
import static frc.robot.Config.Subsystems.INTAKE_CORAL_ENABLED;
import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intakeAlgae.IntakeAlgaeIOMax;
import frc.robot.subsystems.intakeAlgae.IntakeAlgaeIOSim;
import frc.robot.subsystems.intakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.intakeCoral.IntakeCoralIOMax;
import frc.robot.subsystems.intakeCoral.IntakeCoralIOSim;
import frc.robot.subsystems.intakeCoral.IntakeCoralSubsystem;
import lombok.Getter;

public class Rollers extends SubsystemBase {
  @Getter
  private final IntakeAlgaeSubsystem IntakeAlgae =
      INTAKE_ALGAE_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new IntakeAlgaeSubsystem("Feeder", new IntakeAlgaeIOMax())
              : new IntakeAlgaeSubsystem(
                  "Feeder Sim", new IntakeAlgaeIOSim(new DCMotor(1, 1, 1, 1, 1, 1), 0, 0)))
          : null;
  @Getter
  private final IntakeCoralSubsystem IntakeCoral =
          INTAKE_ALGAE_ENABLED
                  ? (MODE == GlobalConstants.RobotMode.REAL
                  ? new IntakeCoralSubsystem("Feeder", new IntakeCoralIOMax())
                  : new IntakeCoralSubsystem(
                  "Feeder Sim", new IntakeCoralIOSim(new DCMotor(1, 1, 1, 1, 1, 1), 0, 0)))
                  : null;

  @Override
  public void periodic() {
    if (INTAKE_ALGAE_ENABLED) IntakeAlgae.periodic();
    if (INTAKE_CORAL_ENABLED) getIntakeCoral().periodic();
  }
}
