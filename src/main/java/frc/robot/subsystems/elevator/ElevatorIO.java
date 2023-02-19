package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double gyroYawRad = 0.0;
    public double targetPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {
  }

  /** Updates the set of loggable inputs. */
  public default void PIDPeriodic(ElevatorIOReal elevatorSparkMax) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  public default void elevatorUp(double speed) {
  }
  public default void elevatorDown(double speed) {
  }

  public default void notElevator(double speed) {
  }

}
