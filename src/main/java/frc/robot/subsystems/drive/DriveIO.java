package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double gyroYawRad = 0.0;
    public double currentRevs = 0.0;
    public double totalRevs = 0.0;
    public double getPitch = 0.0;
    public double targetRevolutions = 0;
    public double leftVoltage = 0.0;
    public double rightVoltage = 0.0;
    public double driveRightPercent = 0.0;
    public double driveLeftPercent = 0.0;
    public double leftEncoderPosition = 0.0;
    public double rightEncoderPosition = 0.0;


    
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {

  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  public default void driveCreep(double speed){};

  public default void slewRate(double forward, double turn){

  }

  public default boolean autoBalancing(double distance){
    return true;
  } 

  public default boolean specificDrive(double distance){
    return false;
  }


  public default boolean specificDriveCharge(double distance){
    return false;
  }

  public default boolean testSpecificDriveCharge(double distance) {
    return false;
  }

  public default void setPosition0(){
    
  }

  public default boolean specificDrive1(double distance, double speed){
    return false;
  }

  public default void drivePercent(double leftPercent, double rightPercent){

  }

  public default double getPitch(){
    return 0.0;
  }
  
}
