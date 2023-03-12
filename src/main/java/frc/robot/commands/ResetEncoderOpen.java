package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;


public class ResetEncoderOpen extends CommandBase{
    private final Intake m_intake;

    public ResetEncoderOpen(Intake intake ){
        m_intake = intake;
        System.out.println("RESET OPEN ENCODER");
        addRequirements(m_intake);
    } 

    @Override
    public void execute(){
        System.out.println("RESET CHOMP ENCODER: EXECUTE");
        m_intake.intakeEncoderOpenReset();
    }

    @Override
    public boolean isFinished() {
        System.out.println("RESET OPEN ENCODER: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
