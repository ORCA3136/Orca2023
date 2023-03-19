package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.commands.StopDrive;

public class ScoreThenBack extends SequentialCommandGroup{
 
    public ScoreThenBack(Drive drivetrain, Intake intake, Elevator elevator)
    {
        addCommands(
            new AutoElevatorPID(58, elevator),
            new AutoPowerElevator(0, elevator),
            new AutoMinivader(Constants.IntakeConstants.miniVaderSpeed, intake),
            new WaitCommand(1),
            new AutoMinivader(0, intake),
            new AutoRunIntake(Constants.IntakeConstants.intakeSloth, intake),
            new WaitCommand(.5),
            new AutoRunIntake(0, intake),
            new AutoMinivader(-1 * Constants.IntakeConstants.miniVaderSpeed, intake),
            new WaitCommand(.5),
            new AutoPowerElevator(- 0.3, elevator),
            new WaitCommand(2),
            new AutoPowerElevator(0, elevator),
            //comment these out if want to go back to bburg
           // new DriveQuickLeft(drivetrain),
           // new WaitCommand(1),
           // new StopDrive(drivetrain),
            //end comment out
            new ChargeAutoDrove(drivetrain, Constants.DrivetrainConstants.kAutoShootThenBack),
            new AutoMinivader(0, intake)
        );

        
    }
    
}
