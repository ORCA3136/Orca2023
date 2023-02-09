package frc.robot.subsystems.limeLight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import org.littletonrobotics.junction.Logger;


public class Limelight extends SubsystemBase 
{
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    private final LimelightIO io;
 


    public void enableLED(String whichLL)
    {
        LimelightHelpers.setLEDMode_ForceOn(whichLL);
    }

    public void disableLED(String whichLL)
    {
        LimelightHelpers.setLEDMode_ForceOff(whichLL);
    }

    
    public Limelight(LimelightIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Limelight", inputs);
  
    }



      
      
  
  }

