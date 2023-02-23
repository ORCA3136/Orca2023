package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase implements DriveIO {
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(6.0);

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);

  /** Creates a new Drive. */
  public Drive(DriveIO io) {
    this.io = io;
  }
  

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    // Update odometry and log the          pose
    odometry.update(new Rotation2d(-inputs.gyroYawRad), getLeftPositionMeters(), getRightPositionMeters());
    Logger.getInstance().recordOutput("Odometry", getPose());
  }

  /** Run open loop at the specified percentage. */
  public void drivePercent(double leftPercent, double rightPercent) {
    io.setVoltage((trueLeft(leftPercent) * 12.0) * 0.3, ((trueRight(rightPercent) * 12.0 )) * 0.3 ) ;
    //if you want to use slew rate uncomment below
    //io.slewRate((trueLeft(leftPercent) * 12.0), ((trueRight(rightPercent) * 12.0 ))  ) ;
  }
  
  public boolean specificDrive(double distance){
    io.specificDrive(distance);
    return true;
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0.0, 0.0);
  }

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the position of the left wheels in meters. */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the position of the right wheels in meters. */
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  public double getLeftVelocityMeters() {
    return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  public double getRightVelocityMeters() {
    return inputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /**
     * Handles deadband of the right stick
     * @param RY
     * @return
     */
  public double trueRight(double right) 
    {
        double stick = right;
        stick *= Math.abs(stick);
        if (Math.abs(stick) < 0.1) {
            stick = 0;
        }
        return stick;
    }
    
    /**
     * handles deadband on the left stick
     * @param LY
     * @return
     */
    public double trueLeft(double left) 
    {
        double stick = left;
        stick *= Math.abs(stick);
        if(Math.abs(stick) < 0.1) {
            stick = 0;
        }
        return stick;

    }
}
