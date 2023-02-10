package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  
  @AutoLog
  public static class IntakeIOInputs {
    public boolean open = false;
  }

/** Updates the set of loggable inputs. */
public default void updateInputs(IntakeIOInputs inputs) {}

public default void open(double speed) {
}
public default void close(double speed) {
}
public default void in() {

}
public default void out() {
  
}

public default void deploy() {

}
public default void retract() {
  
}
public default void stop() {
  
}

public default double getDistance(){
  return 0.0;
}

}

