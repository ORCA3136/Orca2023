package frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSparkMax implements ElevatorIO{
  private final CANSparkMax elevatorMotor1;
  private final CANSparkMax elevatorMotor2;  
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private SparkMaxPIDController m_pidController1;
  private SparkMaxPIDController m_pidController2;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  public ElevatorSparkMax() {

    double targetPosition = 0.0;
    elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevator1, MotorType.kBrushless);
    elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevator2, MotorType.kBrushless);

    leftEncoder = elevatorMotor1.getEncoder();
    rightEncoder = elevatorMotor2.getEncoder();

    elevatorMotor1.restoreFactoryDefaults();
    elevatorMotor2.restoreFactoryDefaults();

    elevatorMotor1.enableVoltageCompensation(12.0);
    elevatorMotor2.enableVoltageCompensation(12.0);
    elevatorMotor1.setSmartCurrentLimit(30);
    elevatorMotor2.setSmartCurrentLimit(30);

    //Note one of these will need to be inverted!!!


    elevatorMotor1.burnFlash();
    elevatorMotor2.burnFlash();

  
 
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController1 = elevatorMotor1.getPIDController();
    m_pidController2 = elevatorMotor2.getPIDController();
    

    // PID coefficients
    kP = 0.1;       
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController1.setP(kP);
    m_pidController1.setI(kI);
    m_pidController1.setD(kD);
    m_pidController1.setIZone(kIz);
    m_pidController1.setFF(kFF);
    m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController2.setP(kP);
    m_pidController2.setI(kI);
    m_pidController2.setD(kD);
    m_pidController2.setIZone(kIz);
    m_pidController2.setFF(kFF);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }
    public void PIDPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController1.setP(p); kP = p; }
    if((i != kI)) { m_pidController1.setI(i); kI = i; }
    if((d != kD)) { m_pidController1.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController1.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController1.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController1.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    if((p != kP)) { m_pidController2.setP(p); kP = p; }
    if((i != kI)) { m_pidController2.setI(i); kI = i; }
    if((d != kD)) { m_pidController2.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController2.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController2.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController2.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }


    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", leftEncoder.getPosition());
    SmartDashboard.putNumber("ProcessVariable", rightEncoder.getPosition());

  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition());
    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition());
    inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        leftEncoder.getVelocity());
    inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        rightEncoder.getVelocity());
    

   // inputs.gyroYawRad = gyro.getYaw();
  }
 
  
    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
      elevatorMotor1.setVoltage(leftVolts);
      elevatorMotor2.setVoltage(rightVolts);
    }

    //NOTES - nowhere does the PID actually execute - so please add in the code that will
    //execute and update the PID controller to the target position


    
};