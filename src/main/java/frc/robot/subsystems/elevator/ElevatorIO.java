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
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;
    public double distance = 0.0;
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

  public default void elevatorPower(double speed) {
  }

  public default void notElevator(double speed) {
  }

  public default double getDistance(){
    return 0.0;
  }

  public default void setDistance(double dist){

  }

  public default void elevatorEncoderReset(){
  
  }

}
