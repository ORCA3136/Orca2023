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
  private double rollTolerance;
  private double speed;
  private double currentPitch;
  private double lastPitch = 0;
  private static final double Pitch_Treshhold = 0.01;


  public AutoBalancingDrive(Drive subsystem, double tolerance) {
    driveTrain = subsystem;
    rollTolerance = tolerance;


    addRequirements(driveTrain);
  }
  
  public void initialize() {
    System.out.println("AUTOBALANCE: INITALIZING");

  }

  public void execute() {
    //Dashboard numbers
    //speed, direction, pitch
    System.out.println("AUTOBALANCE: EXECUTE");


    currentPitch = driveTrain.getPitch();
    speed = Math.sin(Math.toRadians(currentPitch) * DrivetrainConstants.autoBalanceXConstant);

    SmartDashboard.putNumber("Balance Speed", speed);

    //Speed limit
    if (speed < -0.3) {
      speed = -0.3;
    } 
    else if (speed > 0.3) {
      speed = 0.3;
    }

    //
    if (currentPitch > rollTolerance) {
      System.out.println("AUTOBALANCE: > ROLLTOLERANCE" + speed);
      driveTrain.setVoltage(((speed) * 12.0) * DrivetrainConstants.autoBalanceXConstant, (((speed) * 12.0 )) * DrivetrainConstants.autoBalanceXConstant * - 1);         //Change speed
    }
    else if (currentPitch < -rollTolerance) {
      System.out.println("AUTOBALANCE: < ROLLTOLERANCE" + speed);
      driveTrain.setVoltage(((speed) * 12.0) * DrivetrainConstants.autoBalanceXConstant, (((speed) * 12.0) * DrivetrainConstants.autoBalanceXConstant  * -1));
    }
    else {
      System.out.println("AUTOBALANCE: STOP");
      driveTrain.stop();
    }
  }

  public void end(boolean interrupted) {
    System.out.println("AUTOBALANCE: END");


  }
  
  public boolean isFinished() {
    System.out.println("AUTOBALANCE: ISFINISHED");
    return false;
  }
}