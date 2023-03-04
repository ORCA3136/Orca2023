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
import frc.robot.commands.auto.DrivetrainAuto;
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
  private double totalRev = 0.0;
  private double currentRev = 0.0;
  private double getAngle;

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

    boolean autoBalanceXMode;
    boolean autoBalanceYMode;

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

    getAngle = gyro.getPitch();

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
    inputs.currentRevs = currentRev;
    inputs.totalRevs = totalRev;
    inputs.getPitch = gyro.getPitch();
    
    inputs.gyroYawRad = gyro.getYaw();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  public void stopDrive(double left, double right){
    leftLeader.setVoltage(0);
    rightLeader.setVoltage(0);
  }

  @Override
  public void slewRate(double left, double right){

    // Slew-rate limits the forward/backward input, limiting forward/backward acceleration
    if(left>=0 && right >=0)
    {   
      setVoltage(leftFilter.calculate(left), rightFilter.calculate(right));
    }
    else{
      setVoltage(left, right);
      leftFilter.reset(0.0);
      rightFilter.reset(0.0);
    }
  }

  public RelativeEncoder getRightEncoder()
  {
      return leftEncoder;
  }

  public RelativeEncoder getLeftEncoder(){
      return rightEncoder;
  }

  public void drivePercent(double leftPercent, double rightPercent) {
    setVoltage(((leftPercent) * 12.0) * 0.7, (((rightPercent) * 12.0 )) * 0.7 ) ;
    //if you want to use slew rate uncomment below
    //io.slewRate((trueLeft(leftPercent) * 12.0), ((trueRight(rightPercent) * 12.0 ))  ) ;
  }

  public void driveCreep(double speed)
  {
    drivePercent(-speed,speed ) ;
  }
  

  //DONT USE
  public boolean specificDrive(double distance) 
  {
      System.out.println("SpecificDrive>>");
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
      System.out.println("SpecificDrive -beforewhile");
      while(currentRevolutions<totalRevolutions)
      {
          totalRev = totalRevolutions;
          System.out.println("SpecificDrive -inwhile");
          SmartDashboard.putNumber("Current Heading: ", gyro.getAngle());
          SmartDashboard.putNumber("Heading eror: ", error);
          System.out.println("SpecificDrive -beforeif");
/*           if(error<0)
          {
            System.out.println("SpecificDrive -error<0: "+error);
              drivePercent(DrivetrainConstants.kLeftAuto-(kP*error), DrivetrainConstants.kLeftAuto+(kP*error));

          }
          else if(error>0)
          {
            System.out.println("SpecificDrive -error>0: "+error);
              drivePercent(DrivetrainConstants.kLeftAuto+(kP*error), DrivetrainConstants.kRightAuto-(kP*error));

          }
          else
          {*/
          System.out.println("SpecificDrive -else: "+error);
            drivePercent(DrivetrainConstants.kLeftAuto, DrivetrainConstants.kRightAuto);
          //}
          //set the motors to running - comment out for a bit
          //error = startHeading - gyro.getAngle();
          currentRevolutions = (-1*getLeftEncoder().getPosition()) * perRev;
          currentRev = currentRevolutions;
          SmartDashboard.putNumber("Current Revs", currentRevolutions);
          
          SmartDashboard.putNumber("Total Revs", totalRevolutions);
          
      }
      System.out.println("SpecificDrive<<");

      complete = true;

      return complete;
}

    public boolean specificDrive1(double distance){
     // int perRev =  getLeftEncoder().getCountsPerRevolution();
      
      double totalRevolutions = distance;
      double currentRevolutions = 0;
      
      while(currentRevolutions<totalRevolutions)
      {
        drivePercent(-1*DrivetrainConstants.kLeftAuto, -1*DrivetrainConstants.kRightAuto);
        currentRevolutions = (getLeftEncoder().getPosition()) ;
        currentRev = currentRevolutions;
      }

      return true;
      
    }

    public boolean specificDriveCharge(double distance){
      // int perRev =  getLeftEncoder().getCountsPerRevolution();
       
       double totalRevolutions = distance;
       double currentRevolutions = 0;
       
       while(currentRevolutions<totalRevolutions)
       {
         drivePercent(-1*DrivetrainConstants.ChargeAutoLeft, -1*DrivetrainConstants.ChargeAutoRight);
         currentRevolutions = (getLeftEncoder().getPosition()) ;
         currentRev = currentRevolutions;
       }
 
       return true;
       
     }

    public void setPosition0(){
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);

    }

    public boolean autoBalancing(){
        getAngle = gyro.getPitch();
        boolean complete = true;
            if(getAngle > 2)
            { 
              drivePercent(DrivetrainConstants.kLeftAuto, DrivetrainConstants.kRightAuto);
            }
            if (getAngle < 2)
            {
              drivePercent(DrivetrainConstants.kLeftAuto, DrivetrainConstants.kRightAuto);
            }
            else
            {
               stopDrive(0, 0);
            }

            return complete;
    } 

}
