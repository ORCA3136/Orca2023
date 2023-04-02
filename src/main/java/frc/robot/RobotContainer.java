// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ChompPID;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.Minivader;
import frc.robot.commands.OpenIntake;
import frc.robot.commands.PowerElevator;
import frc.robot.commands.ResetElevatorEncoder;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.ResetEncoderOpen;
import frc.robot.commands.RunChomp;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopDrive;
import frc.robot.commands.StraightForward;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.auto.AutoBalancingDrive;
import frc.robot.commands.auto.AutoMove;
import frc.robot.commands.auto.ScoreMidCone;
import frc.robot.commands.auto.ScoreThenBalance;
import frc.robot.commands.auto.ScoreTopCone;
import frc.robot.commands.auto.ScoreTopConeStayStill;
import frc.robot.commands.auto.TestChargeAutoDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.limeLight.Limelight;
import frc.robot.subsystems.limeLight.LimelightIO;
import frc.robot.subsystems.limeLight.LimelightIOReal;
import frc.robot.subsystems.limeLight.LimelightIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  boolean toggle = false;
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Elevator elevator;
  private final Limelight limelight;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick joystick = new Joystick(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
 // private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains  subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(new DriveIOSparkMax());
        intake = new Intake(new IntakeIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        limelight = new Limelight(new LimelightIOReal());
        //flywheel = new Flywheel(new FlywheelIOSparkMax());
        // drive = new Drive(new DriveIOFalcon500());
        // flywheel = new Flywheel(new FlywheelIOFalcon500());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(new DriveIOSim());
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOReal());
        limelight = new Limelight(new LimelightIOSim());
        //flywheel = new Flywheel(new FlywheelIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
        drive = new Drive(new DriveIO() {
        });
        intake = new Intake(new IntakeIO() {
        });
        elevator = new Elevator(new ElevatorIO(){
        });
        limelight = new Limelight(new LimelightIOReal());

        break;
    }

    // Set up auto routines
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addDefaultOption("Shoot Top Cone", new ScoreTopCone(drive, intake, elevator));
    autoChooser.addOption("Shoot Top Cone: Stay Still", new ScoreTopConeStayStill(drive, intake, elevator));
    autoChooser.addOption("Shoot Mid Cone", new ScoreMidCone(drive, intake, elevator));
    autoChooser.addOption("Drive", new AutoMove(drive, intake, elevator));
  //  autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive, flywheel));
    autoChooser.addOption("Score Top Auto Balance", new ScoreThenBalance(drive, intake, elevator));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        new RunCommand(() -> drive.drivePercent(-controller.getLeftY(), controller.getRightY()), drive));
    //elevator.setDefaultCommand(new PowerElevator(0, elevator));
    //BUTTONS FOR XBOX
    //Minidvader in
    controller.rightTrigger().onTrue(new Minivader(-1 * Constants.IntakeConstants.miniVaderSpeed, intake));
    controller.rightTrigger().onFalse(new Minivader(0, intake));
    //minivader out
    controller.leftTrigger().onTrue(new Minivader(Constants.IntakeConstants.miniVaderSpeed, intake));
    controller.leftTrigger().onFalse(new Minivader(0, intake));

    controller.x().onTrue(new PowerElevator(Constants.ElevatorConstants.elevatorSpeed, elevator));
    controller.x().onFalse(new PowerElevator(0, elevator));
    
    controller.y().onTrue(new PowerElevator(-1 * Constants.ElevatorConstants.downelElevatorSpeed, elevator));
    controller.y().onFalse(new PowerElevator(0, elevator));
    //open chomp
    controller.rightBumper().onTrue(new RunChomp(-1 * Constants.IntakeConstants.chompSpeed,intake));
    controller.rightBumper().onFalse(new RunChomp(0,intake));
    //close chomp
    controller.leftBumper().onTrue(new RunChomp(Constants.IntakeConstants.closeChompSpeed,intake));
    controller.leftBumper().onFalse(new RunChomp(0,intake));

    //intake in + Back left
    controller.a().onTrue(new RunIntake(-1 * Constants.IntakeConstants.intakeInSloth, intake));
    controller.a().onFalse(new RunIntake(0, intake));

    //intake out + Back right
    controller.b().onTrue(new RunIntake(Constants.IntakeConstants.intakeSloth, intake));
    controller.b().onFalse(new RunIntake(0, intake));

    controller.start().onTrue(new ResetEncoderOpen(intake));

    controller.back().onTrue(new AutoBalancingDrive(drive, 7));
    controller.back().onFalse(new RunCommand(() -> drive.drivePercent(-controller.getLeftY(), controller.getRightY()), drive));
    
    //BUTTONS FOR JOYSTICK
    JoystickButton joyX = new JoystickButton(joystick, 1); 
    JoystickButton joyA = new JoystickButton(joystick, 2); 
    JoystickButton joyY = new JoystickButton(joystick, 4);
    JoystickButton joyB = new JoystickButton(joystick, 5);
    JoystickButton joyRB = new JoystickButton(joystick, 3);
    JoystickButton joyRT = new JoystickButton(joystick, 6);
    JoystickButton joyLB = new JoystickButton(joystick, 7);
    JoystickButton joyLT = new JoystickButton(joystick, 8);
    JoystickButton joyHome = new JoystickButton(joystick, 10);
    JoystickButton joyBack = new JoystickButton(joystick, 11);
    JoystickButton joyStart = new JoystickButton(joystick, 12);

    //Creep Button 
    joyStart.onTrue(new StraightForward(drive));
    joyStart.onFalse(new RunCommand(() -> drive.drivePercent(-controller.getLeftY(), controller.getRightY()), drive));


    //Reset Encoders Elevator
    joyBack.onTrue(new ResetElevatorEncoder(elevator));

    //Reset Encoders Minivader and Chomp
    joyHome.onTrue(new ResetEncoder(intake));

    //High Cube
    joyX.onTrue(new ElevatorPID(48, elevator));
    joyX.onFalse(new PowerElevator(0,elevator));

    //Mid Cube
    joyA.onTrue(new ElevatorPID(37, elevator));
    joyA.onFalse(new PowerElevator(0,elevator));

    //High Cone lay
    joyLT.onTrue(new ElevatorPID(53, elevator));
    joyLT.onFalse(new PowerElevator(0,elevator));

    //Mid Cone lay
    joyRT.onTrue(new ElevatorPID(39.5, elevator));
    joyRT.onFalse(new PowerElevator(0,elevator));

    //Mid Cone Stand
    joyB.onTrue(new ElevatorPID(45, elevator));
    joyB.onFalse(new PowerElevator(0,elevator));

    //Shelf
    joyY.onTrue(new ElevatorPID(47, elevator));
    joyY.onFalse(new PowerElevator(0,elevator));

    //Elevator down
    joyLB.onTrue(new PowerElevator(-1 * Constants.ElevatorConstants.downelElevatorSpeed, elevator));
    joyLB.onFalse(new PowerElevator(0, elevator));
   
    //Elevator Up
    joyRB.onTrue(new PowerElevator(Constants.ElevatorConstants.elevatorSpeed, elevator));
    joyRB.onFalse(new PowerElevator(0, elevator));


    
  }

  public void resetIntakeEncoders()
  {
    intake.intakeEncoderReset();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
 