package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
  public static final int LEFT_ELEVATOR = 42; // TODO: Change to correct Motor ID's
  public static final boolean LEFT_INVERTED = true;
  public static final int RIGHT_ELEVATOR = 41; // TODO: Change to correct Motor ID's
  public static final boolean RIGHT_INVERTED = true;
  public static final double PULLEY_RADIUS = 0.016;

  public static final boolean isFlex = true; // if the motors on the Elevator are flex motors

  public static final double kP = 0.02; // 0.4 pid for neo
  public static final double kI = 0.0;
  public static final double kD = 0.0;
}
