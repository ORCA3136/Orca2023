package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase{
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Creates a new Drive. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void PIDPeriodic(){

  }

  public void elevatorUp(double speed){
    io.elevatorUp(speed);
  }

  public void elevatorDown(double speed){
    io.elevatorDown(speed);
  }

  public void notElevator(){
    io.notElevator(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);
  }

}
