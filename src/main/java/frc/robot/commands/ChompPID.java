package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.IntakeConstants;


public class ChompPID extends CommandBase{
    private final Intake m_intake;

    private double position = 0.0;
    private double finalPosition = 0.0; 
    public ChompPID(double pos, Intake intake ){
        m_intake = intake;
        position = pos;
        System.out.println("CHOMP PID");
        addRequirements(m_intake);
    } 

    @Override
    public void execute(){
        System.out.println("RUNINTAKE: CHOMPING : "+position);
      finalPosition =  m_intake.setChomper(position) ;
    }

    @Override
    public boolean isFinished() {
        System.out.println("RUNINTAKE: FINISHED POSITION: "+position+" FINAL POSITION: "+finalPosition);
        System.out.println("Calculated: "+Math.abs(position - finalPosition));
        if(Math.abs(position - finalPosition)<2)
        {
            System.out.println("RETURNING TRUE");
            return true;
        }
        else{
            System.out.println("RETURNING FALSE");
            return false;
        }
    }
}
