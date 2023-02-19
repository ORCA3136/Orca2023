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
public default void intakeIn(double speed) {
}
public default void intakeOut(double speed) {
}

public default void deployIntake(double speed) {

}
public default void retractIntake(double speed) {
  
}
public default void stop() {
  
}

public default double getDistance(){
  return 0.0;
}

}

