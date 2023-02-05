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
public final class Constants {
  public static final Mode currentMode = Mode.REAL;




//CAN IDs

public static final int kLeftleader = 2;
public static final int kRightleader = 4;
public static final int kLeftFollwer = 3;
public static final int kRightFollwer = 6;
public static final int elevatorMotor1=7;
public static final int elevatorMotor2=8;
public static final int intakeLeft=9;
public static final int intakeRight=10;
public static final int intakeChomp=11;
public static final int miniVader=12;

//Intake speeds
public static final double chompSpeed=.1;
public static final double miniVaderSpeed=.1;
public static final double intakeSpeed=.1;







  public static enum Mode {
    /** Running on a real robot. */
    REAL,

  
    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
