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
import frc.robot.commands.ResetElevatorEncoder;
import frc.robot.commands.ResetEncoderDrivetrain;
import frc.robot.commands.RunChomp;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopDrive;

public class ScoreThenBalance extends SequentialCommandGroup{
 
    public ScoreThenBalance(Drive drivetrain, Intake intake, Elevator elevator)
    {
        addCommands(
            new AutoElevatorPID(58, elevator),
            new AutoPowerElevator(0, elevator),
            new AutoMinivader(Constants.IntakeConstants.miniVaderSpeed, intake),
            new WaitCommand(.5),
            new AutoMinivader(0, intake),
            new AutoRunIntake(Constants.IntakeConstants.intakeSloth, intake),
            new WaitCommand(.5),
            new AutoRunIntake(0, intake),
            new AutoMinivader(-1 * Constants.IntakeConstants.miniVaderSpeed, intake),
            new WaitCommand(.5),
            new AutoPowerElevator(- 0.3, elevator),
            new WaitCommand(1.5),
            new AutoPowerElevator(0, elevator),
            new DrivetrainAuto(drivetrain, Constants.DrivetrainConstants.kAutoConeBalanceDistance, Constants.DrivetrainConstants.kLeftAuto),
            new ResetEncoderDrivetrain(drivetrain),
            new ResetEncoderDrivetrain(drivetrain),
            new WaitCommand(.5),
            new DrivetrainAuto(drivetrain, Constants.DrivetrainConstants.kAutoShootThenBackThenForwardTest, -1 * Constants.DrivetrainConstants.kLeftAuto),
            new AutoBalancingDrive(drivetrain, 7),
            new AutoMinivader(0, intake)
        );

        
    }
    
}
