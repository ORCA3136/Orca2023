package frc.robot.subsystems.drive;

import com.google.flatbuffers.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  private final SimpleMotorFeedforward leftModel, rightModel;

  //Creates a SlewRateLimiter that limits the rate of change of the signal to X units per second
  SlewRateLimiter leftFilter = new SlewRateLimiter(DrivetrainConstants.slewRate); 
  SlewRateLimiter rightFilter = new SlewRateLimiter(DrivetrainConstants.slewRate); 
  
  
  private double lastLeftVelocityMPS = 0.0;
  private double lastRightVelocityMPS = 0.0;
  private double afterEncoderReduction = 6.0; // Internal encoders'

  //private final Pigeon2 gyro;

  public DriveIOSparkMax() {
    leftLeader = new CANSparkMax(DrivetrainConstants.kLeftleader, MotorType.kBrushless);
    rightLeader = new CANSparkMax(DrivetrainConstants.kRightleader, MotorType.kBrushless);
    leftFollower = new CANSparkMax(DrivetrainConstants.kLeftFollower, MotorType.kBrushless);
    rightFollower = new CANSparkMax(DrivetrainConstants.kRightFollower, MotorType.kBrushless);

    boolean autoBalanceXMode;
    boolean autoBalanceYMode;

    leftModel = new SimpleMotorFeedforward(0.20554, 0.10965, 0.016329);
    rightModel = new SimpleMotorFeedforward(0.20231, 0.11768, 0.0085871);

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

    
    leftLeader.setOpenLoopRampRate(.3);
    rightLeader.setOpenLoopRampRate(.3);

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

    /**
     * For the charge station 
     */
    public boolean specificDriveCharge(double distance){
      // int perRev =  getLeftEncoder().getCountsPerRevolution();
       
       double totalRevolutions = distance;
       double currentRevolutions = 0;
       
       while(currentRevolutions<totalRevolutions+1)
       {
         drivePercent(-1*DrivetrainConstants.ChargeAutoLeft, -1*DrivetrainConstants.ChargeAutoRight);
         currentRevolutions = (getLeftEncoder().getPosition()) ;
         currentRev = currentRevolutions;
       }
       
       while(currentRevolutions>totalRevolutions+2)
       {
         drivePercent(DrivetrainConstants.ChargeAutoLeft*.5, DrivetrainConstants.ChargeAutoRight*.5);
         currentRevolutions = (getLeftEncoder().getPosition()) ;
         currentRev = currentRevolutions;
       }
       return true;
       
     }

    public void setPosition0(){
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);

    }

    @Override
    public void drivePercentPID(double leftPercent, double rightPercent){
      driveVelocity(leftPercent * DrivetrainConstants.MAX_VELOCITY_MPS, rightPercent * DrivetrainConstants.MAX_VELOCITY_MPS);

    }
     /**
     * takes the speed and works to conver it all to volts since volts is what we really need / want for kinematics.
     * TODO - get the output values on shuffleboard so I can understand what is going on!!!!
     * 
     * @param leftVelocityMPS
     * @param rightVelocityMPS
     */
    public void driveVelocity(double leftVelocityMPS, double rightVelocityMPS)
    {

        SmartDashboard.putNumber("LEFT MPS", leftVelocityMPS);
        SmartDashboard.putNumber("RIGHT MPS", rightVelocityMPS);
        SmartDashboard.putNumber("GYRO ANGLE", gyro.getAngle());
        double maxAccelerationPerCycle = Double.POSITIVE_INFINITY * DrivetrainConstants.loopPeriodSecs;

        double leftAcceleration = lastLeftVelocityMPS > 0 
        ? leftVelocityMPS - lastLeftVelocityMPS 
        : lastLeftVelocityMPS - leftVelocityMPS;
        //Shuffleboard.getTab("Drive Details").add("LEFT ACCELERATION", leftAcceleration);

        if(leftAcceleration> maxAccelerationPerCycle)
        {
            lastLeftVelocityMPS += leftVelocityMPS > 0 ? maxAccelerationPerCycle : -maxAccelerationPerCycle;
        }
        else
        {
            lastLeftVelocityMPS = leftVelocityMPS;
        }

        double rightAcceleration = lastRightVelocityMPS > 0 
        ? rightVelocityMPS - lastRightVelocityMPS 
        : lastRightVelocityMPS - rightVelocityMPS;
       // Shuffleboard.getTab("Drive Details").add("RIGHT ACCELERATION", rightAcceleration);

        if(rightAcceleration> maxAccelerationPerCycle)
        {
            lastRightVelocityMPS += rightVelocityMPS > 0 ? maxAccelerationPerCycle : -maxAccelerationPerCycle;
        }
        else
        {
            lastRightVelocityMPS = rightVelocityMPS;
        }

        //calculate the setpoint and the feed forward voltage
        double leftVelocityRPS = lastLeftVelocityMPS / DrivetrainConstants.WHEEL_RADIUS_METERS;
        double rightVelocityRPS = lastRightVelocityMPS / DrivetrainConstants.WHEEL_RADIUS_METERS;
       // Shuffleboard.getTab("Drive Details").add("LEFT VELOCITY RPS", leftVelocityRPS);
      //  Shuffleboard.getTab("Drive Details").add("RIGHT VELOCITY RPS", rightVelocityRPS);

        double leftFFVolts = leftModel.calculate(leftVelocityRPS);
        double rightFFVolts = rightModel.calculate(rightVelocityRPS);

     //   Shuffleboard.getTab("Drive Details").add("LEFT FF Volts", leftFFVolts);
    //    Shuffleboard.getTab("Drive Details").add("RIGHT FF Volts", rightFFVolts);
        SmartDashboard.putNumber("LEFT FF VOLTS", leftFFVolts);
        SmartDashboard.putNumber(" RIGHT FF VOLTS", rightFFVolts);

        //this is just a basic drive -
        //leftLeader.setVoltage(leftFFVolts);
        //rightLeader.setVoltage(rightFFVolts);

        //this is a pid drive
        double leftRPM = Units.radiansPerSecondToRotationsPerMinute(leftVelocityRPS) * afterEncoderReduction;
        double rightRPM = Units.radiansPerSecondToRotationsPerMinute(rightVelocityRPS) * afterEncoderReduction;
        SmartDashboard.putNumber("LEFT RPM", leftRPM);
        SmartDashboard.putNumber("RIGHT RPM", rightRPM);

        //   Shuffleboard.getTab("Drive Details").add("LEFT RPM", leftRPM);
     //   Shuffleboard.getTab("Drive Details").add("RIGHT RPM", rightRPM);

        leftLeader.getPIDController().setReference(leftRPM, ControlType.kVelocity, 0, leftFFVolts,ArbFFUnits.kVoltage);
        rightLeader.getPIDController().setReference(rightRPM, ControlType.kVelocity, 0, rightFFVolts,ArbFFUnits.kVoltage);
    }

}
