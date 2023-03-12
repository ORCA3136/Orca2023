package frc.robot.subsystems.drive;

import com.google.flatbuffers.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
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
  private double getAngle;
  private double error;


  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.trackWidthMeters);


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

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);
    
    leftLeader.setOpenLoopRampRate(.2);
    rightLeader.setOpenLoopRampRate(.2);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);
    leftLeader.setSmartCurrentLimit(30);
    rightLeader.setSmartCurrentLimit(30);

    leftLeader.burnFlash();
    rightLeader.burnFlash();
    leftFollower.burnFlash();
    rightFollower.burnFlash();

    getAngle = gyro.getPitch();
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

  public RelativeEncoder getRightEncoder()
  {
      return leftEncoder;
  }

  public RelativeEncoder getLeftEncoder(){
      return rightEncoder;
  }

  public void drivePercent(double leftPercent, double rightPercent) {
    setVoltage(((leftPercent) * 12.0) * 0.7, (((rightPercent) * 12.0 )) * 0.7 ) ;
  }

  public void driveCreep(double speed)
  {
    drivePercent(speed, speed);
  }

    public boolean specificDrive(double distance){
      double totalRevolutions = distance;
      double currentRevolutions = 0;
      
      while(currentRevolutions<totalRevolutions)
      {
        drivePercent(DrivetrainConstants.kLeftAuto, DrivetrainConstants.kRightAuto);
        currentRevolutions = (getLeftEncoder().getPosition()) ;
        currentRev = currentRevolutions;
      }

      return true;
      
    }

    public boolean specificDriveCharge(double distance){
      // int perRev =  getLeftEncoder().getCountsPerRevolution();
       
       double totalRevolutions = distance;
       double currentRevolutions = 0;
       
       while(currentRevolutions<totalRevolutions+1)
       {
         drivePercent(DrivetrainConstants.ChargeAutoLeft, DrivetrainConstants.ChargeAutoRight);
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
   public boolean autoBalancing(double speed){
    getAngle = gyro.getPitch();
    boolean complete = true;
    System.out.println("AUTOBALANCING");
    error = Math.abs(getAngle) - 0;

        while(getAngle > 2)
        { 
          getAngle = gyro.getPitch();
          System.out.println("> 2");
          driveCreep(-1 * DrivetrainConstants.kBalanceSpeed * getAngle);
        }
        //while(getAngle < -2)
        //{
        //  System.out.println("< -2");
        //  driveCreep(0.3);
        //}
        if(getAngle <= 2)
        {
          getAngle = gyro.getPitch();
          System.out.println("STOP DRIVE");
           stopDrive(0, 0);
        }

        return complete;
} 

  public double getPitch(){
    return gyro.getPitch();
  }

  public double getYaw(){
    return gyro.getYaw();
  }

}
