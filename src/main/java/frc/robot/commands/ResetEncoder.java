package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;


public class ResetEncoder extends CommandBase{
    private final Intake m_intake;

    public ResetEncoder(Intake intake ){
        m_intake = intake;
        System.out.println("RESET ENCODER");
        addRequirements(m_intake);
    } 

    @Override
    public void execute(){
        System.out.println("RESET CHOMP ENCODER: EXECUTE");
        m_intake.intakeEncoderReset();
    }

    @Override
    public boolean isFinished() {
        System.out.println("RESET ENCODER: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
