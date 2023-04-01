package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class ScoreMidCone extends SequentialCommandGroup{
 
    public ScoreMidCone(Drive drivetrain, Intake intake, Elevator elevator)
    {
        addCommands(
            new AutoElevatorPID(39, elevator),
            new AutoPowerElevator(0, elevator),
            new AutoMinivader(Constants.IntakeConstants.miniVaderSpeed, intake),
            new WaitCommand(1),
            new AutoMinivader(0, intake),
            new AutoRunIntake(Constants.IntakeConstants.intakeSloth, intake),
            new WaitCommand(.5),
            new AutoRunIntake(0, intake),
            new AutoPowerElevator(-.3, elevator),
            new WaitCommand(1.5),
            new AutoPowerElevator(0, elevator),
            new AutoMinivader(-1 * Constants.IntakeConstants.miniVaderSpeed, intake),
            new WaitCommand(.5),
            new DrivetrainAuto(drivetrain, Constants.DrivetrainConstants.kAutoConeDistance, DrivetrainConstants.kLeftAuto),
            new AutoMinivader(0, intake)
        
        );

        
    }
    
}
