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
import frc.robot.commands.StopDrive;
import frc.robot.commands.auto.DrivetrainAuto;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.Drive;



public class DriveIOSparkMax implements DriveIO {
  private static final double GEAR_RATIO = 9.40;

  private final CANSparkMax leftLeader;
  private final CANSparkMax rightLeader;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private double totalRev = 0.0;
  private double currentRev = 0.0;
  private double leftVoltage = 0.0;
  private double rightVoltage = 0.0;
  private double driveLeftPercent = 0.0;
  private double driveRightPercent = 0.0;
  private double leftPosition;
  private double rightPosition;
  private double targetRevolutions;

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private double getAngle = gyro.getAngle();


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

    
    leftLeader.setOpenLoopRampRate(.15);
    rightLeader.setOpenLoopRampRate(.15);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);
    leftLeader.setSmartCurrentLimit(30);
    rightLeader.setSmartCurrentLimit(30);

    leftLeader.burnFlash();
    rightLeader.burnFlash();
    leftFollower.burnFlash();
    rightFollower.burnFlash();

    getAngle = gyro.getPitch();

    targetRevolutions = DrivetrainConstants.ChargeRevolutions;

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
    inputs.leftVoltage = leftVoltage;
    inputs.rightVoltage = rightVoltage;
    inputs.driveLeftPercent = driveLeftPercent;
    inputs.driveRightPercent = driveRightPercent;
    inputs.leftEncoderPosition = leftPosition;
    inputs.rightEncoderPosition = rightPosition;

    inputs.gyroYawRad = gyro.getYaw();

    inputs.targetRevolutions = targetRevolutions;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftVoltage = leftVolts;
    rightVoltage = rightVolts;
    //System.out.println("SetVoltage SparkMax: LEFT: "+leftVolts+" RIGHT: "+rightVolts);
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
    driveLeftPercent = leftPercent;
    driveRightPercent = rightPercent;
    //System.out.println("DrivePercent SparkMax LEFT: "+((leftPercent) * 12.0) * DrivetrainConstants.driveSpeed+" RIGHT: "+(((rightPercent) * 12.0 )) * DrivetrainConstants.driveSpeed);

    setVoltage(((leftPercent) * 12.0) * DrivetrainConstants.driveSpeed, (((rightPercent) * 12.0 )) * DrivetrainConstants.driveSpeed);
    //if you want to use slew rate uncomment below
    //io.slewRate((trueLeft(leftPercent) * 12.0), ((trueRight(rightPercent) * 12.0 ))  ) ;
  }

  public void driveCreep(double speed)
  {
    drivePercent(-speed,speed ) ;
  }
  

    public boolean specificDrive1(double distance, double speed){
     // int perRev =  getLeftEncoder().getCountsPerRevolution();
      
      double totalRevolutions = distance;
      double currentRevolutions = 0;

      if(totalRevolutions>0)
      {
        //drives backward since total revolutions are greater than 0, meaning that distance values should be positive
        while(currentRevolutions<totalRevolutions)
        {
          drivePercent(-1*speed, speed);
          currentRevolutions = (getLeftEncoder().getPosition()) ;
          currentRev = currentRevolutions;
        }
        return true;
      }
      else if(totalRevolutions<=0)
      {
        //drives forward since total revolutions are less than 0, meaning that distance values should be negative
        while(currentRevolutions>totalRevolutions)
        {
          drivePercent(-1*speed, speed);
          currentRevolutions = (getLeftEncoder().getPosition()) ;
          currentRev = currentRevolutions;
        }
        return true;
      }
      else{
        return true;
      }
      
    }

    public boolean specificDriveCharge(double distance){
      // int perRev =  getLeftEncoder().getCountsPerRevolution();
       
       double totalRevolutions = distance;
       double currentRevolutions = 0;

       //int flat = 0;
       
       while(currentRevolutions<totalRevolutions+1)
       {
         drivePercent(-1*DrivetrainConstants.ChargeAuto, DrivetrainConstants.ChargeAuto);
         currentRevolutions = (getLeftEncoder().getPosition()) ;
         currentRev = currentRevolutions;
         /*
         if (currentRevolutions >= totalRevolutions && (gyro.getAngle() > -3 && gyro.getAngle() < 3)) {
          flat++;
          if (flat >= 10){
           return true;
          }
         }*/
       }

       while(currentRevolutions>totalRevolutions+2)
       {
         drivePercent(DrivetrainConstants.ChargeAuto*.5, -1*DrivetrainConstants.ChargeAuto*.5);
         currentRevolutions = (getLeftEncoder().getPosition()) ;
         currentRev = currentRevolutions;
         /*
         if (currentRevolutions >= totalRevolutions && (gyro.getAngle() > -3 && gyro.getAngle() < 3)) {
          flat++;
          if (flat >= 10){
           return true;
          }
         }*/
       }
       return true;
       
     }

     public boolean testSpecificDriveCharge(double distance){
      // int perRev =  getLeftEncoder().getCountsPerRevolution();

       double totalRevolutions = distance;
       double currentRevolutions = 0;

       targetRevolutions = DrivetrainConstants.ChargeRevolutions;
       double currentAngle = gyro.getPitch();
       

       while (currentAngle < -2 || currentAngle > 2){
        currentAngle = gyro.getPitch();
        //while 0 < 1
        while(currentRevolutions<totalRevolutions + targetRevolutions)
        {
          //System.out.println("TestCharge: Less than  CurRev: " + currentRevolutions + "  TarRev: " + targetRevolutions);
          if (currentAngle > 3 || currentAngle < -3){
            drivePercent(-1*DrivetrainConstants.ChargeAutoTest*0.5, DrivetrainConstants.ChargeAutoTest*0.5);
          }
          else{
            drivePercent(-1*DrivetrainConstants.SlowChargeAutoTest*0.5, DrivetrainConstants.SlowChargeAutoTest*0.5);
          }
          
          currentRevolutions = (getLeftEncoder().getPosition()) ;
          currentRev = currentRevolutions;
          currentAngle = gyro.getPitch();

          if (currentAngle > 3) {
            System.out.println("TestCharge: Less than angle");
            targetRevolutions -= 0.5;
          }
        }
        
        //while 0 > 2
        while(currentRevolutions>totalRevolutions + targetRevolutions + 1)
        {
          //System.out.println("TestCharge: More than  CurRev: " + currentRevolutions + "  TarRev: " + targetRevolutions);
          if (currentAngle < -3 || currentAngle > 3){
          drivePercent(DrivetrainConstants.ChargeAutoTest*.5, -1*DrivetrainConstants.ChargeAutoTest*.5);
          }
          else {
            drivePercent(DrivetrainConstants.SlowChargeAutoTest*.5, -1*DrivetrainConstants.SlowChargeAutoTest*.5);
          }
          currentRevolutions = (getLeftEncoder().getPosition()) ;
          currentRev = currentRevolutions;
          currentAngle = gyro.getPitch();

          if (currentAngle < -3) {
            System.out.println("TestCharge: More than angle");
            targetRevolutions += 0.5;
          }
        }
        System.out.println("TESTCHARGE: Finished Moving");
      }
       return true;
       
     }

    public void setPosition0(){
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);

      leftPosition = leftEncoder.getPosition();
      rightPosition = rightEncoder.getPosition();
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

    @Override
    public double getPitch(){
      return gyro.getPitch();
    }

}
