package frc.robot.subsystems.limeLight;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {

    @AutoLog
    public static class LimelightIOInputs {
      public double tx = 0.0;
      public double ty = 0.0;
      public double ta = 0.0;
      public boolean ledStatus = true;
    }
    
    public default void updateInputs(LimelightIOInputs inputs) {
    } 
}
