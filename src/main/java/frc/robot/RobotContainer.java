// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
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
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Elevator elevator;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
 // private final Joystick joystick = new Joystick(1);
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
        elevator = new Elevator(new ElevatorSparkMax());
        //flywheel = new Flywheel(new FlywheelIOSparkMax());
        // drive = new Drive(new DriveIOFalcon500());
        // flywheel = new Flywheel(new FlywheelIOFalcon500());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(new DriveIOSim());
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorSparkMax());
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
        //flywheel = new Flywheel(new FlywheelIO() {
        //});
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
    intake.setDefaultCommand(new RunCommand(()->intake.stop(), intake));

    controller.a().toggleOnTrue(new RunCommand(() ->drive.drivePercent(.5,.5),drive));
    controller.b().whileTrue(new RunCommand(() ->drive.stop(),drive));

    controller.x().whileTrue(new RunCommand(() -> intake.open(),intake ));
    controller.y().whileTrue(new RunCommand(()-> intake.close(), intake));

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
    
    //joystick.getRawButton(1)
    //.whileTrue (new InstantCommand(() -> elevator.up(), elevator));
    










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
