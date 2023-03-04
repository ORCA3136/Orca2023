package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;

public class StopDrive extends CommandBase {
    // The subsystem the command runs on
    private Drive driveTrain;
    private boolean complete = false;
  
    public StopDrive(Drive subsystem) {
      driveTrain = subsystem;
      addRequirements(driveTrain);
    }
    
    public void initialize() {
      driveTrain.setPosition0();
    }
  
    public void execute() {
      System.out.println("AUTODRIVE: EXECUTING");
      driveTrain.drivePercent(0, 0);
    }
  
    public void end(boolean interrupted) {
      driveTrain.stop();
      System.out.println("AUTODRIVE: END");
    }
  
    
    public boolean isFinished() {
      driveTrain.stop();
      System.out.println("AUTODRIVE: END");
      return complete;
    }
  }
