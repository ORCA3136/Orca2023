package frc.robot.subsystems.intake;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    
        chompEncoder = chomp.getEncoder();
        leftEncoder = intakeMotor1.getEncoder();
        rightEncoder = intakeMotor2.getEncoder();
        vaderEncoder = miniVader.getEncoder();
            
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
        chomp.setSmartCurrentLimit(40,40);

        chomp.burnFlash();
        intakeMotor1.burnFlash();
        intakeMotor2.burnFlash();
        miniVader.burnFlash();

    

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
  chomp.set(speed);
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
