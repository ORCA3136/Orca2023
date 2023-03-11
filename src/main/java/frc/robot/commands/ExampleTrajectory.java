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
    
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DrivetrainConstants.ksVolts,
                DrivetrainConstants.kvVoltSecondsPerMeter,
                DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
                DrivetrainConstants.kDriveKinematics,
            10);

      TrajectoryConfig config =
          new TrajectoryConfig(
                    DrivetrainConstants.kMaxSpeedMetersPerSecond,
                    DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DrivetrainConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

      Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);
        
       Drive m_drive;

       public ExampleTrajectory(Drive drive, Trajectory exampleTrajectory)
       {
         super(
               exampleTrajectory,
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
