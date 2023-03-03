// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class AutoElevatorPID extends PIDCommand {
  /** Creates a new ReplaceMePIDCommand. */
  public AutoElevatorPID(double distance, Elevator elevator) {
    super(
        // The controller that the command will use
        new PIDController(ElevatorConstants.elevatorkP, ElevatorConstants.elevatorkI, ElevatorConstants.elevatorkD),
        // This should return the measurement
        elevator::getDistance,
        // This should return the setpoint (can also be a constant)
        distance,
        // This uses the output
        output-> elevator.elevatorPower(output*ElevatorConstants.pidThrottle),
        elevator);
        getController().setTolerance(ElevatorConstants.kPositionTolerance);
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
