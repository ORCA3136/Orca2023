package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.Minivader;
import frc.robot.commands.PowerElevator;
import frc.robot.commands.RunChomp;
import frc.robot.commands.RunIntake;

public class ScoreTopCone extends SequentialCommandGroup{
 
    public ScoreTopCone(Drive drivetrain, Intake intake, Elevator elevator)
    {
        addCommands(
            new ElevatorPID(55, elevator),
            new PowerElevator(0, elevator),
            new Minivader(Constants.IntakeConstants.miniVaderSpeed, intake),
            new Minivader(0, intake),
            new RunIntake(Constants.IntakeConstants.intakeSloth, intake),
            new RunIntake(0, intake),
            new Minivader(-1 * Constants.IntakeConstants.miniVaderSpeed, intake),
            new Minivader(0, intake),
            new ElevatorPID(0, elevator),
            new PowerElevator(0, elevator),
            new DriveForward(drivetrain, 10)









            
        
        );


    }
    
}
