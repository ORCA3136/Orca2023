package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
  public static class IntakeIOInputs {

    public boolean open = false;
    public double pressure = 0.0;
    public boolean compressor = false;

  }

/** Updates the set of loggable inputs. */
public default void updateInputs(IntakeIOInputs inputs) {}

public default void open() {
}
public default void close() {
}

}

