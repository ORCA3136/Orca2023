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

  public void elevatorPower(double speed){
    io.elevatorPower(speed);
  }

  public void notElevator(){
    io.notElevator(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);


  }

  public double getDistance()
  {
      return io.getDistance();
  }

  public void setDistance(double dist)
  {
     io.setDistance(dist);
  }

}
