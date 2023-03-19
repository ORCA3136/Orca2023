package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSparkMax;

import javax.swing.text.AbstractDocument.LeafElement;

import edu.wpi.first.wpilibj.XboxController;


/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written
 * explicitly for pedagogical purposes. Actual code should inline a command this
 * simple with {@link edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class DriveQuickRight extends CommandBase {
  // The subsystem the command runs on
  private Drive driveTrain;
  private DriveIOSparkMax driveSpark;
  private boolean complete = false;

  public DriveQuickRight(Drive subsystem) {
    driveTrain = subsystem;
    addRequirements(driveTrain);
  }
  
  public void initialize() {
    driveTrain.setPosition0();
  }

  public void execute() {
    System.out.println("quickright: EXECUTING");
    driveTrain.drivePercent(0,DrivetrainConstants.QuickTurnPwr);
  }

  public void end(boolean interrupted) {

    driveTrain.setPosition0();
    System.out.println("Quickrifght: END");
  }

  
  public boolean isFinished() {

    System.out.println("quickright: END");
    return complete;
  }
}