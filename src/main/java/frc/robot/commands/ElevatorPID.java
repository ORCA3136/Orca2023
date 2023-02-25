// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorPID extends PIDCommand {
  /** Creates a new ReplaceMePIDCommand. */
  public ElevatorPID(double distance, Elevator elevator) {
    super(
        // The controller that the command will use
        new PIDController(ElevatorConstants.elevatorkP, ElevatorConstants.elevatorkI, ElevatorConstants.elevatorkD),
        // This should return the measurement
        elevator::getDistance,
        // This should return the setpoint (can also be a constant)
        distance,
        // This uses the output
        output-> elevator.elevatorUp(output),
        elevator);
        getController().setTolerance(IntakeConstants.kOpenTolerance);
          // Use the output here
        }
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
