package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.Constants.ElevatorConstants;


public class PowerElevator extends CommandBase{
    private final Elevator m_elevator;

    private double m_power = 0.0;

    public PowerElevator(double power, Elevator elevator ){
        m_elevator = elevator;
        m_power = power;
        System.out.println("POWERELEVATOR");
        addRequirements(m_elevator);
    } 

    @Override
    public void initialize(){
        System.out.println("POWERELEATOR: INITIALIZED");
        m_elevator.elevatorPower(m_power);
    }

    @Override
    public boolean isFinished() {
        System.out.println("POWERELEATOR: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
