package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger Idle() {
    return button(1);
  }

  @Override
  public Trigger LevelOne() {
    return button(2);
  }

  @Override
  public Trigger LevelTwo() {
    return button(3);
  }

  @Override
  public Trigger LevelThree() {
    return button(4);
  }

  @Override
  public Trigger LevelFour() {
    return button(5);
  }

  @Override
  public Trigger Intake() {
    return button(6);
  }

  @Override
  public Trigger Outake() {
    return button(7);
  }

  @Override
  public Trigger Testing() {
    return button(8);
  }

  @Override
  public Trigger WantsCoral() {
    return button(9);
  }

  @Override
  public Trigger WantsAlgae() {
    return button(10);
  }

  @Override
  public Trigger Source() {
    return button(11);
  }
}
