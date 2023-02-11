package frc.robot.subsystems.intake;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.IntakeConstants;;

public class IntakeIOReal implements IntakeIO {
  //  private static final double GEAR_RATIO = 1.5;
    private boolean isOpen = false;

    private double distance;
    Encoder counter;

    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;  
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private VictorSPX chomp;


        /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
        

    public IntakeIOReal() {
       // leftSide = new CANSparkMax(IntakeConstants.intakeLeft, MotorType.kBrushless);
       // rightSide = new CANSparkMax(IntakeConstants.intakeRight, MotorType.kBrushless);
      //  miniVader = new CANSparkMax(IntakeConstants.miniVader, MotorType.kBrushless);
        chomp = new VictorSPX(IntakeConstants.intakeChomp);
        intakeMotor1 = new CANSparkMax(IntakeConstants.intakeLeft, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.intakeRight, MotorType.kBrushless);
    
        leftEncoder = intakeMotor1.getEncoder();
        rightEncoder = intakeMotor2.getEncoder();
    
        intakeMotor1.restoreFactoryDefaults();
        intakeMotor2.restoreFactoryDefaults();
    
        intakeMotor1.enableVoltageCompensation(12.0);
        intakeMotor2.enableVoltageCompensation(12.0);
        intakeMotor1.setSmartCurrentLimit(30);
        intakeMotor2.setSmartCurrentLimit(30);
    
        //Note one of these will need to be inverted!!!
    
    
        intakeMotor1.burnFlash();
        intakeMotor2.burnFlash();
    


        //counter for the intake chomper - will act as our encoder
        //counter = new Counter(Counter.Mode.kTwoPulse);
        counter = new Encoder(1, 2, true, Encoder.EncodingType.k4X);
        counter.setDistancePerPulse(1.0/44.4);
        //counter.setMaxPeriod(.1);
        



        
       // encoder = leftSide.getEncoder();
       // pid = leftSide.getPIDController();
        

       // leftSide.restoreFactoryDefaults();
       // rightSide.restoreFactoryDefaults();

       // rightSide.follow(leftSide);
       // rightSide.setInverted(true);

       // leftSide.burnFlash();
       // rightSide.burnFlash();
    }
//MAIN INTAKE FUNCTIONS

public void IntakeInny(double flywheelspeed){
  intakeMotor1.set(flywheelspeed);
  intakeMotor2.set(flywheelspeed);
}
public void IntakeOuty(double flywheelspeed){
  intakeMotor1.set(-1 * flywheelspeed);
  intakeMotor2.set(-1 * flywheelspeed);
}

public void IntakeDeployey(double miniVaderSpeed){
  intakeMotor1.set(miniVaderSpeed);
  intakeMotor2.set(miniVaderSpeed);
}
public void IntakeRetractey(double miniVaderSpeed){
  intakeMotor1.set(-1 * miniVaderSpeed);
  intakeMotor2.set(-1 * miniVaderSpeed);
}

public void IntakeOpen(){
}
public void IntakeClose(){
}
public void stop(){
  chomp.set(ControlMode.PercentOutput,0);
  //counter.get();
  //System.out.println("STOP COUNT: "+counter.get());
 // leftSide.stopMotor();
 // rightSide.stopMotor();
 intakeMotor1.set(0);
 intakeMotor2.set(0);
}

public void open(double speed)
{
  chomp.set(ControlMode.PercentOutput,speed);
  isOpen = true;
  System.out.println("OPEN COUNT : "+counter.get());
  System.out.println("DIRECTION: "+ counter.getDirection());
  System.out.println("RATE: "+ counter.getRate());
  System.out.println("DISTANCE: "+ counter.getDistance());
  setDistance(counter.getDistance());
}

public void close(double speed)
{
  chomp.set(ControlMode.PercentOutput, -1*speed);
  isOpen = false;
  System.out.println("CLOSE COUNT: "+counter.get());
  System.out.println("DIRECTION: "+ counter.getDirection());
  System.out.println("RATE: "+ counter.getRate());
  System.out.println("DISTANCE: "+ counter.getDistance());
  setDistance(counter.getDistance());
}


  @Override
  public void updateInputs(IntakeIOInputs inputs) 
  {
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
