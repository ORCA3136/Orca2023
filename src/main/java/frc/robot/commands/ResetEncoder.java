package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;


public class ResetEncoder extends CommandBase{
    private final Intake m_intake;

    public ResetEncoder(Intake intake ){
        m_intake = intake;
        System.out.println("POWERELEVATOR");
        addRequirements(m_intake);
    } 

    @Override
    public void initialize(){
        System.out.println("POWERELEATOR: INITIALIZED");
        m_intake.intakeEncoderReset();
    }

    @Override
    public boolean isFinished() {
        System.out.println("POWERELEATOR: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
