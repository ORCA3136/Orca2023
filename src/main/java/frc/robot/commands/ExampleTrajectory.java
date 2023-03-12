package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;

public class ExampleTrajectory extends RamseteCommand{

       Drive m_drive;

       public ExampleTrajectory(Drive drive)
       {
         super(
               DrivetrainConstants.exampleTrajectory,
               drive::getPose,
               new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
               new SimpleMotorFeedforward(
               DrivetrainConstants.ksVolts,
               DrivetrainConstants.kvVoltSecondsPerMeter,
               DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
               DrivetrainConstants.kDriveKinematics,
               drive::getWheelSpeeds,
               new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
               new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
               // RamseteCommand passes volts to the callback
               drive::setVoltage,
               drive);
         m_drive = drive;

       }


       public void end(boolean interrupted) {
        m_drive.setVoltage(0, 0); //stop it
      }


}
