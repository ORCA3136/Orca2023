// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
public final class Constants 
{
  public static final Mode currentMode = Mode.REAL;

//DRIVE TRAIN
public static final class DrivetrainConstants
{
  public static final int kLeftleader = 1;
  public static final int kRightleader = 2;
  public static final int kLeftFollower = 3;
  public static final int kRightFollower = 4;

  public static final double slewRate = 4.0;
  public static final double kLeftAuto = -.2;
  public static final double kRightAuto = .2;

  public static final double kAutoDistance = 5;
}

//ELEVATOR
public static final class ElevatorConstants
{
  public static final int kElevator1=6;
  public static final int kElevator2=7;

  public static final double elevatorSpeed = 0.3;
  public static final double downelElevatorSpeed = 0.1;

  public static final double elevatorkP = 0.1; 
  public static final double elevatorkI = 1e-1;
  public static final double elevatorkD = .01; 
}

//INTAKE CONSTANTS
public static final class IntakeConstants
{
  public static final int intakeLeft=8;
  public static final int intakeRight=9;
  public static final int intakeChomp=10;
  public static final int miniVader=5;
  public static final double chompSpeed= .4;

  public static final double miniVaderSpeed=.3;
  public static final double intakeSpeed=.1;

  public static final double chompkP = 0.6; 
  public static final double chompkI = 1e-4;
  public static final double chompkD = .01; 
  public static final double kOpenTolerance = 0.1;

  public static final double intakeSloth = 0.8;
  public static final double intakeZoom = 0.5;

  
}

  public static final class ButtonConstants
  {
    public static final int kY = 4;
    public static final int kX = 1;
    public static final int kA = 2;
    public static final int kB = 3;
    public static final int kLB = 5;
    public static final int kRB = 6;
    public static final int kRT = 8;
    public static final int kLT = 7;
    public static final int kSelect = 11;
    public static final int kStart = 12;
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
