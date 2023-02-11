package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }
    @Override
    public void periodic() 
    {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Intake", inputs);
  
    }
    public void IntakeInny(double speed)
    {
        io.IntakeInny(speed);
    }
    public void IntakeOuty(double speed)
    {
        io.IntakeOuty(speed);
    }

    public void open(double speed)
    {
        io.open(speed);
    }

    public void close(double speed)
    {
        io.close(speed);
    }

    public void deploy(double speed)
    {
        io.IntakeDeployey(speed);
    }

    public void retract(double speed)
    {
        io.IntakeRetractey(speed);
    }
    public void stop()
    {
        io.stop();
    }

    public double getDistance()
    {
        return io.getDistance();
    }
}

