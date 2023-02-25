package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;


/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written
 * explicitly for pedagogical purposes. Actual code should inline a command this
 * simple with {@link edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoBalancingDrive extends CommandBase {
  // The subsystem the command runs on
  private Drive driveTrain;
  private DriveIOSparkMax driveSpark;
  private boolean complete = false;
  private double driveDist = DrivetrainConstants.kAutoDistance;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private double getAngle;


  public AutoBalancingDrive(Drive subsystem, double distance) {
    getAngle = gyro.getPitch();

  }
  
  public void initialize() {

  }

  public void execute() {
    getAngle = gyro.getPitch();
    if(getAngle > 2){
        driveTrain.drivePercent(DrivetrainConstants.kLeftAuto, DrivetrainConstants.kRightAuto); 
    }

    if(getAngle < 2){
        driveTrain.drivePercent(DrivetrainConstants.kLeftAuto, DrivetrainConstants.kRightAuto);
    }

    else{
        driveTrain.stop();
    }

  }

  public void end(boolean interrupted) {

  }

  
  public boolean isFinished() {

    return complete;
  }
}