package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Counter;
import frc.robot.Constants.IntakeConstants;;

public class IntakeIOReal implements IntakeIO {
  //  private static final double GEAR_RATIO = 1.5;
    private boolean isOpen = false;
    Counter counter;

    
    private final VictorSPX chomp;
   // private final CANSparkMax leftSide;
  //  private final CANSparkMax rightSide;
  //  private final RelativeEncoder encoder;
  //  private final SparkMaxPIDController pid;
  //  private final CANSparkMax miniVader;


    public IntakeIOReal() {
       // leftSide = new CANSparkMax(IntakeConstants.intakeLeft, MotorType.kBrushless);
       // rightSide = new CANSparkMax(IntakeConstants.intakeRight, MotorType.kBrushless);
      //  miniVader = new CANSparkMax(IntakeConstants.miniVader, MotorType.kBrushless);
        chomp = new VictorSPX(IntakeConstants.intakeChomp);
        counter = new Counter(Counter.Mode.kTwoPulse);
        counter.setUpSource(1);
        counter.setDownSource(2);
    
        // Set the decoding type to 2X
        counter.setUpSourceEdge(true, true);
        counter.setDownSourceEdge(true, true);
        counter.setDistancePerPulse(1.0/44.4);
        counter.setMaxPeriod(.1);


        
       // encoder = leftSide.getEncoder();
       // pid = leftSide.getPIDController();
        

       // leftSide.restoreFactoryDefaults();
       // rightSide.restoreFactoryDefaults();

       // rightSide.follow(leftSide);
       // rightSide.setInverted(true);

       // leftSide.burnFlash();
       // rightSide.burnFlash();
    }

  private void enableVoltageCompensation() {
    }

//MAIN INTAKE FUNCTIONS

public void IntakeIn(){
}
public void IntakeOut(){
}
public void IntakeDeploy(){
}
public void IntakeRetract(){
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
}

public void open()
{
  chomp.set(ControlMode.PercentOutput,-1 * .3);
  isOpen = true;
  System.out.println("OPEN COUNT : "+counter.get());
  System.out.println("DIRECTION: "+ counter.getDirection());
  System.out.println("RATE: "+ counter.getRate());

}

public void close()
{
  chomp.set(ControlMode.PercentOutput, .3);
  isOpen = false;
  System.out.println("CLOSE COUNT: "+counter.get());
  System.out.println("DIRECTION: "+ counter.getDirection());
  System.out.println("RATE: "+ counter.getRate());
}




public void setVelocity(double velocityRadPerSec, double ffVolts) {
  //  pid.setReference(
  //      Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
  //          * GEAR_RATIO,
  //      ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }


  @Override
  public void updateInputs(IntakeIOInputs inputs) 
  {
      inputs.open = isOpen;

  }

  public void configurePID(double kP, double kI, double kD) {
  //  pid.setP(kP, 0);
  //  pid.setI(kI, 0);
  //  pid.setD(kD, 0);
  //  pid.setFF(0, 0);
  }

}
