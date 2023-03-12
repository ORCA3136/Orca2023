// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  // DRIVE TRAIN
  public static final class DrivetrainConstants {
    public static final int kLeftleader = 1;
    public static final int kRightleader = 2;
    public static final int kLeftFollower = 3;
    public static final int kRightFollower = 4;

    public static final double slewRate = 4.0;

    public static final double kLeftAuto = .3;
    public static final double kRightAuto = .3;

    public static final double kCreepForwardRight = -0.3;
    public static final double kCreepForwardLeft = -0.3;


    public static final double kLeftChargeAuto = .5;
    public static final double kRightChargeAuto = .5;

    public static final double kCreepButton = .3;

    public static final double kBalanceDampener = 0.1;
    public static final double kBalanceSpeed = 0.02;

    public static final double LeftAuto = .5;
    public static final double RightAuto = .5;

    public static final double ChargeAutoLeft = .6;
    public static final double ChargeAutoRight = .6;

    public static final double kAutoDistance = 85;
    public static final double kAutoConeDistance = 85;
    public static final double kAutoMidConeDistance = 70;
    public static final double kAutoShootThenBack = 47;
    public static final double kAutoChargeTest = 44;


    public static final double ksVolts = 0.36099;
    public static final double kvVoltSecondsPerMeter = 4.9929;
    public static final double kaVoltSecondsSquaredPerMeter = 2.7487;
    public static final double kPDriveVel = 0.000009487;
    public static final double trackWidthMeters = .514;

    //Theoretical Max drivetrain velocity is 15.81 ft/s or 4.818 m/s
    public static final double kMaxSpeedMetersPerSecond = 0.25;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics =new DifferentialDriveKinematics(DrivetrainConstants.trackWidthMeters);

    public static final PathPlannerTrajectory traj = PathPlanner.loadPath("New Path", 2, 2);

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DrivetrainConstants.ksVolts,
            DrivetrainConstants.kvVoltSecondsPerMeter,
            DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            DrivetrainConstants.kDriveKinematics,
        10);

        public static final TrajectoryConfig config =
    new TrajectoryConfig(
              DrivetrainConstants.kMaxSpeedMetersPerSecond,
              DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DrivetrainConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

    public static final Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(),
              new Pose2d(4, 0, new Rotation2d(0)),
          // Pass config
          config);


  }

  // ELEVATOR
  public static final class ElevatorConstants {
    public static final int kElevator1 = 6;
    public static final int kElevator2 = 7;

    public static final double elevatorSpeed = 0.3;
    public static final double downelElevatorSpeed = 0.3;

    public static final double elevatorkP = 0.07;
    public static final double elevatorkI = 1e-2;
    public static final double elevatorkD = .0;
    public static final double kPositionTolerance = 1;
    public static final double pidThrottle = 0.5; // use this to cap speed for pid controller

    public static final double autoPIDThrottle = 0.3; // use this to cap speed for pid controller


  }

  // INTAKE CONSTANTS
  public static final class IntakeConstants {
    public static final int intakeLeft = 8;
    public static final int intakeRight = 9;
    public static final int intakeChomp = 10;

    public static final int miniVader = 5;
    public static final double chompSpeed = .7;
    public static final double closeChompSpeed = .5;

    public static final double miniVaderSpeed = .5;
    public static final double intakeSpeed = .1;

    public static final double chompkP = 0.6;
    public static final double chompkI = 1e-4;
    public static final double chompkD = .01;
    public static final double kOpenTolerance = 0.1;

    public static final double intakeSloth = 0.4;
    public static final double fastAsPossible = 1;

    public static final double intakeInSloth = 0.3;
    public static final double intakeZoom = 0.5;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
