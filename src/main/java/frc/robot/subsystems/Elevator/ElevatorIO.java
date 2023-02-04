package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double gyroYawRad = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void PIDPeriodic(ElevatorSparkMax elevatorSparkMax) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

}
