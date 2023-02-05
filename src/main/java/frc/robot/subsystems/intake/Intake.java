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
    public void periodic() {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Intake", inputs);
  
    }
    public void in()
    {
        io.in();
    }
    public void out()
    {
        io.out();
    }

    public void open()
    {
        io.open();
    }

    public void close()
    {
        io.close();
    }

}
