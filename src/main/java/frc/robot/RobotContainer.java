// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Minivader;
import frc.robot.commands.OpenIntake;
import frc.robot.commands.PowerElevator;
import frc.robot.commands.RunChomp;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SpinAuto;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
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
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Spin", new SpinAuto(drive));
  //  autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive, flywheel));

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
  //  intake.setDefaultCommand(new RunCommand(()->intake.stop(), intake));
    //elevator.setDefaultCommand(new RunCommand(()->elevator.notElevator(),elevator));

   // controller.a().toggleOnTrue(new RunCommand(() ->drive.drivePercent(.5,.5),drive));
   // controller.b().whileTrue(new RunCommand(() ->drive.stop(),drive));
                                                                                                                                       
    //controller.x().whileTrue(new RunCommand(() -> intake.open(IntakeConstants.chompSpeed),intake ));
    //controller.y().whileTrue(new RunCommand(()-> intake.close(IntakeConstants.chompSpeed), intake));

    controller.a().onTrue(new Minivader(-1 * Constants.IntakeConstants.miniVaderSpeed, intake));
    controller.a().onFalse(new Minivader(0, intake));

    controller.b().onTrue(new Minivader(Constants.IntakeConstants.miniVaderSpeed, intake));
    controller.b().onFalse(new Minivader(0, intake));

    //controller.b().whileTrue(new RunCommand(() ->intake.intakeOut(IntakeConstants.intakeSloth),intake));

    controller.x().onTrue(new PowerElevator(Constants.ElevatorConstants.elevatorSpeed, elevator));
    controller.x().onFalse(new PowerElevator(0, elevator));
    
    controller.y().onTrue(new PowerElevator(-1 * Constants.ElevatorConstants.elevatorSpeed, elevator));
    controller.y().onFalse(new PowerElevator(0, elevator));

    controller.rightTrigger().onTrue(new RunIntake(Constants.IntakeConstants.intakeSloth, intake));
    controller.rightTrigger().onFalse(new RunIntake(0, intake));

    controller.leftTrigger().onTrue(new RunIntake(-1 * Constants.IntakeConstants.intakeSloth, intake));
    controller.leftTrigger().onFalse(new RunIntake(0, intake));


  //  if (joystick.getRawButtonPressed(1)) { 
  //    intake.IntakeInny(IntakeConstants.intakeSloth);
  // } else {
  //    intake.stop();
  // }
  //
  // if (joystick.getRawButtonPressed(2)) {
  //  intake.IntakeOuty(IntakeConstants.intakeSloth);
  // } else {
  //  intake.stop();
  // }

  // if (joystick.getRawButtonPressed(3)) {
  //  intake.deploy(IntakeConstants.intakeSloth);
  // } else {
  //  intake.stop();
  // }

   //if (joystick.getRawButtonPressed(4)) {
   // intake.retract(IntakeConstants.intakeSloth);
   //} else {
   // intake.stop();
   //}  
   //low g
   //if (joystick.getRawButtonPressed(5)) {
   // elevator.moveitMoveitElevatorUp(ElevatorConstants.elevatorSpeed);
   //} else {
   // intake.stop();
   //}
   //clmb brake
   //if (joystick.getRawButtonPressed(6)) {
   // elevator.moveitMoveitElevatorDown(ElevatorConstants.elevatorSpeed);
   //} else {
   // intake.stop();
   //}  


    controller.rightBumper().onTrue(new RunChomp(Constants.IntakeConstants.chompSpeed,intake));
    controller.leftBumper().onTrue(new RunChomp(-1 * Constants.IntakeConstants.chompSpeed, intake));
    //intake related buttons
    //A Opens the jaws
    //B closes the jaws
    //LB deploys the jaws
    //RB retracts the jaws
    //LT atracts
    //RT repels
    /**   controller.a()
        .whileTrue(new InstantCommand(() -> intake.open(), intake));
    controller.b()
        .whileTrue(new InstantCommand(() -> intake.close(), intake));
    controller.leftBumper()
        .whileTrue(new InstantCommand(() -> intake.deploy(), intake));
    controller.rightBumper()
        .whileTrue(new InstantCommand(() -> intake.retract(), intake));
    controller.leftTrigger()
        .whileTrue(new InstantCommand(() -> intake.in(), intake));
    controller.rightTrigger()
        .whileTrue(new InstantCommand(() -> intake.out(), intake));
    controller.x()
        .whileTrue(new InstantCommand(() -> intake.stop(), intake));
        */
    //Eleveator related buttons
    
    if (joystick.getRawButtonPressed(1)) {
      if (toggle) {
         // Current state is true so turn off
         elevator.elevatorUp(ElevatorConstants.elevatorSpeed);
         toggle = false;
      } else {
         // Current state is false so turn on
         elevator.elevatorDown(ElevatorConstants.elevatorSpeed);
         toggle = true;
      }
   }







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
 