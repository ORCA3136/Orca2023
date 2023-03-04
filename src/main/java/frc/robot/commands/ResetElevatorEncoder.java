package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;


public class ResetElevatorEncoder extends CommandBase{
    private final Elevator m_elevator;

    public ResetElevatorEncoder(Elevator elevator ){
        m_elevator = elevator;
        System.out.println("RESET ENCODER");
        addRequirements(m_elevator);
    } 

    @Override
    public void execute(){
        System.out.println("RESET CHOMP ENCODER: EXECUTE");
        m_elevator.elevatorEncoderReset();
    }

    @Override
    public boolean isFinished() {
        System.out.println("RESET ENCODER: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
