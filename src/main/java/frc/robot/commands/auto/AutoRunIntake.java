package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.IntakeConstants;


public class AutoRunIntake extends CommandBase{
    private final Intake m_intake;

    private double m_power = 0.0;

    public AutoRunIntake(double power, Intake intake ){
        m_intake = intake;
        m_power = power;
        System.out.println("AUTORUNINTAKE");
        addRequirements(m_intake);
    } 

    @Override
    public void execute(){
        System.out.println("RUNINTAKE: INITIALIZED : "+m_power);
        m_intake.intakeWheelPower(m_power);
    }

    @Override
    public boolean isFinished() {
        System.out.println("RUNINTAKE: FINISHED");
        // TODO Auto-generated method stub
        return true;
    }
}
