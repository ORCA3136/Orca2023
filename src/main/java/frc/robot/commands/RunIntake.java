package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.IntakeConstants;


public class RunIntake extends CommandBase{
    private final Intake m_intake;

    private double m_power = 0.0;

    public RunIntake(double power, Intake intake ){
        m_intake = intake;
        m_power = power;
        System.out.println("RUNINTAKE");
        addRequirements(m_intake);
    } 

    @Override
    public void initialize(){
        System.out.println("RUNINTAKE: INITIALIZED : "+m_power);
        m_intake.intakeIn(m_power);
    }

    @Override
    public boolean isFinished() {
        System.out.println("RUNINTAKE: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
