package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import javax.swing.text.AbstractDocument.LeafElement;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;


/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written
 * explicitly for pedagogical purposes. Actual code should inline a command this
 * simple with {@link edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class Balance extends CommandBase {
  // The subsystem the command runs on
  private Drive driveTrain;
  private DriveIOSparkMax driveSpark;
  private boolean complete = false;
  private double balanceSpeed = DrivetrainConstants.kCreepButton;
  private double currentAngle;
  private int timesRun;


  public Balance(Drive subsystem, double speed) {
    SmartDashboard.putNumber("Drive Distance", DrivetrainConstants.kAutoDistance);
    speed = balanceSpeed;
    driveTrain = subsystem;
    addRequirements(driveTrain);
    currentAngle = driveTrain.getPitch();

  }
  
  public void initialize() {

  }

  public void execute() {
    currentAngle = driveTrain.getPitch();
    timesRun = 0;
    while(Math.abs(currentAngle) > 2 && timesRun<200){
        System.out.println("BALANCE: EXECUTING");
        timesRun += 1;
        currentAngle = driveTrain.getPitch();
        if(currentAngle > 2){
            driveTrain.drivePercent(-1 *Math.abs(currentAngle) * 0.8 * DrivetrainConstants.kBalanceDampener, -1 * Math.abs(currentAngle) * 0.5 * DrivetrainConstants.kBalanceDampener);
        }

        else if(currentAngle < -2){
            driveTrain.drivePercent(Math.abs(currentAngle) * 0.8 * DrivetrainConstants.kBalanceDampener, Math.abs(currentAngle) * 0.5 * DrivetrainConstants.kBalanceDampener);
        }

        else{
            driveTrain.stop();
        }
    }

  }

  public void end(boolean interrupted) {
    System.out.println("BALANCE: END");
  }

  
  public boolean isFinished() {
    System.out.println("BALANCE: FINISHED " + complete);
    if(Math.abs(currentAngle) < 2 ){
        return true;
    }
    else{
        return false;
    }
  }
}
