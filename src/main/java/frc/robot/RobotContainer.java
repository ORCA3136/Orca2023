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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.Minivader;
import frc.robot.commands.OpenIntake;
import frc.robot.commands.PowerElevator;
import frc.robot.commands.RunChomp;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.auto.AutoMove;
import frc.robot.commands.auto.ScoreTopCone;
import frc.robot.commands.auto.SpinAuto;
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
    autoChooser.addOption("Spin", new SpinAuto(drive));
    autoChooser.addOption("Shoot Cone", new ScoreTopCone(drive, intake, elevator));
    autoChooser.addDefaultOption("Drive", new AutoMove(drive, intake, elevator));
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
    //elevator.setDefaultCommand(new PowerElevator(0, elevator));
    //BUTTONS FOR XBOX

    controller.start().onTrue(new TurnToTarget(drive));

    controller.a().onTrue(new Minivader(-1 * Constants.IntakeConstants.miniVaderSpeed, intake));
    controller.a().onFalse(new Minivader(0, intake));

    controller.b().onTrue(new Minivader(Constants.IntakeConstants.miniVaderSpeed, intake));
    controller.b().onFalse(new Minivader(0, intake));

    controller.x().onTrue(new PowerElevator(Constants.ElevatorConstants.elevatorSpeed, elevator));
    controller.x().onFalse(new PowerElevator(0, elevator));
    
    controller.y().onTrue(new PowerElevator(-1 * Constants.ElevatorConstants.downelElevatorSpeed, elevator));
    controller.y().onFalse(new PowerElevator(0, elevator));

    controller.rightTrigger().onTrue(new RunIntake(Constants.IntakeConstants.intakeSloth, intake));
    controller.rightTrigger().onFalse(new RunIntake(0, intake));

    controller.leftTrigger().onTrue(new RunIntake(-1 * Constants.IntakeConstants.intakeSloth, intake));
    controller.leftTrigger().onFalse(new RunIntake(0, intake));

    //CLOSE CHOMPER
    controller.rightBumper().onTrue(new RunChomp(Constants.IntakeConstants.chompSpeed,intake));
    controller.rightBumper().onFalse(new RunChomp(0,intake));

    //OPEN CHOMPER
    controller.leftBumper().onTrue(new RunChomp(-1 * Constants.IntakeConstants.chompSpeed,intake));
    controller.leftBumper().onFalse(new RunChomp(0,intake));

    controller.start().onTrue(new ElevatorPID(40, elevator));
    controller.start().onFalse(new PowerElevator(0,elevator));

    
    //BUTTONS FOR JOYSTICK
    JoystickButton joystickButton1 = new JoystickButton(joystick, 1); 
    JoystickButton joystickButton2 = new JoystickButton(joystick, 2); 
    JoystickButton joystickButton3 = new JoystickButton(joystick, 3);
    JoystickButton joystickButton4 = new JoystickButton(joystick, 4);

    //UNKNOWN Button
    joystickButton4.onTrue(new ElevatorPID(0.5, elevator));
    joystickButton4.onFalse(new ElevatorPID(0, elevator));

    //X Button
    joystickButton1.onTrue(new PowerElevator(-1 * Constants.ElevatorConstants.downelElevatorSpeed, elevator));
    joystickButton1.onFalse(new PowerElevator(0, elevator));

    //A Button
    joystickButton2.onTrue(new PowerElevator(Constants.ElevatorConstants.elevatorSpeed, elevator));
    joystickButton2.onFalse(new PowerElevator(0, elevator));

    //UNKOWN Button
    joystickButton3.onTrue(new RunChomp(-1 * Constants.IntakeConstants.chompSpeed, intake));
    joystickButton3.onFalse(new RunChomp(0, intake));

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
 