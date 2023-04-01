package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;


public class ResetEncoderDrivetrain extends CommandBase{
    private final Drive m_drivetrain;

    public ResetEncoderDrivetrain(Drive drive ){
        m_drivetrain = drive;
        System.out.println("RESET ENCODER");
        addRequirements(m_drivetrain);
    } 

    @Override
    public void initialize(){
        System.out.println("POWERELEATOR: INITIALIZED");
        m_drivetrain.setPosition0();
    }

    @Override
    public void execute(){
        System.out.println("RESET CHOMP ENCODER: EXECUTE");
        m_drivetrain.setPosition0();
    }

    @Override
    public boolean isFinished() {
        System.out.println("RESET ENCODER: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
