package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class AutoMove extends SequentialCommandGroup{
 
    public AutoMove(Drive drivetrain, Intake intake, Elevator elevator)
    {
        addCommands(
            new DrivetrainAuto(drivetrain, DrivetrainConstants.kAutoDistance)













        
        );


    }
    
}
