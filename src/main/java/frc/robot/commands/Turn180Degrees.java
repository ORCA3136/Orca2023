package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;


public class Turn180Degrees extends CommandBase{
    private final Drive m_drivetrain;

    public Turn180Degrees(Drive drive ){
        m_drivetrain = drive;
        System.out.println("RESET ENCODER");
        addRequirements(m_drivetrain);
    } 

    @Override
    public void initialize(){
        System.out.println("POWERELEATOR: INITIALIZED");
    }

    @Override
    public void execute(){
        System.out.println("RESET CHOMP ENCODER: EXECUTE");
    }

    @Override
    public boolean isFinished() {
        System.out.println("RESET ENCODER: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
