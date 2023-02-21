package frc.robot.subsystems.drive;

import com.google.flatbuffers.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.Drive;



public class DriveIOSparkMax implements DriveIO {
  private static final double GEAR_RATIO = 10.96;

  private final CANSparkMax leftLeader;
  private final CANSparkMax rightLeader;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;


  private final AHRS gyro = new AHRS(SPI.Port.kMXP);


  //Creates a SlewRateLimiter that limits the rate of change of the signal to X units per second
  SlewRateLimiter leftFilter = new SlewRateLimiter(DrivetrainConstants.slewRate); 
  SlewRateLimiter rightFilter = new SlewRateLimiter(DrivetrainConstants.slewRate); 

  //private final Pigeon2 gyro;

  public DriveIOSparkMax() {
    leftLeader = new CANSparkMax(DrivetrainConstants.kLeftleader, MotorType.kBrushless);
    rightLeader = new CANSparkMax(DrivetrainConstants.kRightleader, MotorType.kBrushless);
    leftFollower = new CANSparkMax(DrivetrainConstants.kLeftFollower, MotorType.kBrushless);
    rightFollower = new CANSparkMax(DrivetrainConstants.kRightFollower, MotorType.kBrushless);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeader.setInverted(true);
    rightLeader.setInverted(true);
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);
    leftLeader.setSmartCurrentLimit(30);
    rightLeader.setSmartCurrentLimit(30);

    leftLeader.burnFlash();
    rightLeader.burnFlash();
    leftFollower.burnFlash();
    rightFollower.burnFlash();

   // gyro = new Pigeon2(0);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
    inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        leftEncoder.getVelocity() / GEAR_RATIO);
    inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        rightEncoder.getVelocity() / GEAR_RATIO);
   // inputs.gyroYawRad = gyro.getYaw();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void slewRate(double left, double right){

    // Slew-rate limits the forward/backward input, limiting forward/backward acceleration
    setVoltage(leftFilter.calculate(left), rightFilter.calculate(right));
  }

  public RelativeEncoder getRightEncoder()
  {
      return leftEncoder;
  }

  public RelativeEncoder getLeftEncoder(){
      return rightEncoder;
  }
  
  public boolean specificDrive(double distance) 
  {
      double kP = 0.05;
      double startHeading = gyro.getAngle();
      
      //double error = startHeading - gyro.getAngle();
      double error = 0; //set this for now so it only drives
      boolean complete = false;
      getLeftEncoder().setPosition(0); //set the position to 0
      Double leftPosition = getLeftEncoder().getPosition();
      SmartDashboard.putNumber("Left Enc Pos: ", leftPosition);
      SmartDashboard.putNumber("Start Heading ", startHeading);

      //really only need to get this once...
      int perRev =  getLeftEncoder().getCountsPerRevolution();
      double totalRevolutions = distance*perRev;
      double currentRevolutions = 0;
      while(currentRevolutions<totalRevolutions)
      {
          SmartDashboard.putNumber("Current Heading: ", gyro.getAngle());
          SmartDashboard.putNumber("Heading eror: ", error);
          if(error<0)
          {
              drivePercent(frc.robot.Constants.DrivetrainConstants.kLeftAuto-(kP*error), frc.robot.Constants.DrivetrainConstants.kLeftAuto+(kP*error));

          }
          else if(error>0)
          {
              drivePercent(frc.robot.Constants.DrivetrainConstants.kLeftAuto+(kP*error), frc.robot.Constants.DrivetrainConstants.kRightAuto-(kP*error));

          }
          else
          {
              drivePercent(frc.robot.Constants.DrivetrainConstants.kLeftAuto, frc.robot.Constants.DrivetrainConstants.kRightAuto);
          }
          //set the motors to running - comment out for a bit
          //error = startHeading - gyro.getAngle();
          currentRevolutions = (-1*getLeftEncoder().getPosition()) * perRev;
          SmartDashboard.putNumber("Current Revs", currentRevolutions);
          
          SmartDashboard.putNumber("Total Revs", totalRevolutions);
      }
      complete = true;

      return complete;
}

}
