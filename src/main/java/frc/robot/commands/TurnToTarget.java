// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurnToTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;
  private boolean alignedToTarget = false;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private double Kp = -0.1;  // Proportional control constant

  private boolean done = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToTarget(Drive subsystem) {
    drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    //LIMELIGHT

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //read values periodically
     table = NetworkTableInstance.getDefault().getTable("limelight");
     tx = table.getEntry("tx");
     ty = table.getEntry("ty");
     ta = table.getEntry("ta");
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);

     double steeringAdjust = Kp * x;
    if(Math.abs(x)<.2)
    {
      done=true;
    }
    else
    {
      done = false;
    }
    double left=steeringAdjust;
    double right=-steeringAdjust;
   
    drive.drivePercent(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}