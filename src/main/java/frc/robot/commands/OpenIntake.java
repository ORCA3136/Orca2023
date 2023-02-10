package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class OpenIntake extends PIDCommand {
    public OpenIntake(double distance, Intake intake)
    {
        super(
        new PIDController(IntakeConstants.chompkP,IntakeConstants.chompkI, IntakeConstants.chompkD),
        intake::getDistance,
        distance,
        output-> intake.open(output),
        intake);
        getController().setTolerance(IntakeConstants.kOpenTolerance);
    }

    @Override
    public boolean isFinished() {
      System.out.println("STOPPED OPEN INTAKE "+getController().atSetpoint());
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
