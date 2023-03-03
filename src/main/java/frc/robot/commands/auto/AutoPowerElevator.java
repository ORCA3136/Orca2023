package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.Constants.ElevatorConstants;


public class AutoPowerElevator extends CommandBase{
    private final Elevator m_elevator;

    private double m_power = 0.0;

    public AutoPowerElevator(double power, Elevator elevator ){
        m_elevator = elevator;
        m_power = power;
        System.out.println("AUTOPOWERELEVATOR");
        addRequirements(m_elevator);
    } 

    @Override
    public void execute(){
        System.out.println("AUTOPOWERELEATOR: EXECUTE");
        m_elevator.elevatorPower(m_power);
    }

    @Override
    public boolean isFinished() {
        System.out.println("POWERELEATOR: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
