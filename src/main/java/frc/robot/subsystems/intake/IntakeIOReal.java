package frc.robot.subsystems.intake;


import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Minivader;

public class IntakeIOReal implements IntakeIO {
    private boolean isOpen = false;

    private double distance;
    Encoder counter;

    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
    private final CANSparkMax miniVader;  
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder vaderEncoder;
    private final RelativeEncoder chompEncoder;
    private final CANSparkMax chomp;
    private double getPosition;
    private SparkMaxPIDController chompPID;
    private double chompP, chompI, chompD, chompFF, chompIz, chompMinOutput, chompMaxOutput, chompMaxVel, chompMaxAccel, chompError;

        /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
        

    public IntakeIOReal() {
        chomp = new CANSparkMax(IntakeConstants.intakeChomp, MotorType.kBrushless);
        miniVader = new CANSparkMax(IntakeConstants.miniVader, MotorType.kBrushless);
        intakeMotor1 = new CANSparkMax(IntakeConstants.intakeLeft, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.intakeRight, MotorType.kBrushless);
    
            
        chomp.restoreFactoryDefaults();
        miniVader.restoreFactoryDefaults();
        intakeMotor1.restoreFactoryDefaults();
        intakeMotor2.restoreFactoryDefaults();
    
        intakeMotor1.enableVoltageCompensation(12.0);
        intakeMotor2.enableVoltageCompensation(12.0);

        intakeMotor1.setSmartCurrentLimit(30,40);
        intakeMotor2.setSmartCurrentLimit(30,40);

        miniVader.enableVoltageCompensation(12.0);
        miniVader.setSmartCurrentLimit(30,40);

        chomp.enableVoltageCompensation(12.0);
        chomp.setSmartCurrentLimit(20,40);

        chomp.setIdleMode(IdleMode.kBrake);

        chomp.burnFlash();
        intakeMotor1.burnFlash();
        intakeMotor2.burnFlash();
        miniVader.burnFlash();

        chompEncoder = chomp.getEncoder();
        leftEncoder = intakeMotor1.getEncoder();
        rightEncoder = intakeMotor2.getEncoder();
        vaderEncoder = miniVader.getEncoder();


        chompPID = chomp.getPIDController();

       // chompPID.setSmartMotionMaxVelocity(chompMaxVel, smartMotionSlot);
       // chompPID.setSmartMotionMaxAccel(chompMaxAccel, smartMotionSlot);

        robotInit();
        
    }
    public void robotInit() {
        chompEncoder.setPosition(0.0);
        vaderEncoder.setPosition(0.0);
      }
    

public double setChomper(double position)
{
  chompP =0.03;
  chompI = 0;
  chompD = 0;;
  chompMaxOutput=.3;
  chompMinOutput = -.3;
        
  
  chompPID.setP(chompP);
  chompPID.setI(chompI);
  chompPID.setD(chompD);
//chompPID.setIZone(chompIz);
  //chompPID.setFF(chompFF);
  chompPID.setOutputRange(chompMinOutput, chompMaxOutput);
  REVLibError error = chompPID.setReference(position,CANSparkMax.ControlType.kPosition);
  System.out.print("ERROR: "+error.toString());
  return chompEncoder.getPosition();
}
        
public boolean isMiniVaderIn(){
  getPosition = vaderEncoder.getPosition();
  if(getPosition > 2)
  { 
    return true;
  }
  else
  {
    return false;
  }
}     

public boolean isMiniVaderOut(){
  getPosition = vaderEncoder.getPosition();
  if(getPosition < 2)
  { 
    return true;
  }
  else
  {
    return false;
  }
}     



public void intakeWheelPower(double power){
  intakeMotor1.set(power);
  intakeMotor2.set(-1 * power);
}

public void miniVaderPower(double miniVaderSpeed){
  miniVader.set(miniVaderSpeed);
}

public void IntakeOpen(){
}
public void IntakeClose(){
}
public void stop(){
  chomp.set(0);
  //counter.get();
  //System.out.println("STOP COUNT: "+counter.get());
 // leftSide.stopMotor();
 // rightSide.stopMotor();
 intakeMotor1.set(0);
 intakeMotor2.set(0);
}

public void open(double speed)
{
  chomp.set(speed);
  isOpen = true;
  System.out.println("OPEN COUNT : "+chompEncoder.getPosition());
  System.out.println("RATE: "+ chompEncoder.getVelocity());
  //System.out.println("DISTANCE: "+ chompEncoder.getPosition());
  setDistance(chompEncoder.getPosition());
}

public void open1 (double speed){
  //check the chomp encoder position- if it becomes positive stop!
  double currentPosition = chompEncoder.getPosition();
  if(currentPosition >0 && speed <0)
  {
    chomp.set(speed);
  }
  else if(currentPosition<=0) {//allow it to open or close as it is negative
    chomp.set(speed);
  }
  else if(currentPosition>0 && speed>0)
  {
    chomp.set(0.0);
  }
  else{
    System.out.println("NOT SURE WHAT IS HAPPENING: CURRENT POSITION: "+currentPosition+" SPEED: "+speed);
    chomp.set(0.0);
  }
}

public void close(double speed)
{
  chomp.set(speed);
  isOpen = false;
  System.out.println("OPEN COUNT : "+chompEncoder.getPosition());
  System.out.println("RATE: "+ chompEncoder.getVelocity());
  //System.out.println("DISTANCE: "+ chompEncoder.getPosition());
  setDistance(chompEncoder.getPosition());
}


  @Override
  public void updateInputs(IntakeIOInputs inputs) 
  {
    inputs.getPosition = vaderEncoder.getPosition();
    inputs.open = isOpen;
    inputs.chompPosition = chompEncoder.getPosition();

  }

  @Override
  public double getDistance()
  {
    return distance;
  }

  public void setDistance(double dist)
  {
    distance = dist;
  }

}
