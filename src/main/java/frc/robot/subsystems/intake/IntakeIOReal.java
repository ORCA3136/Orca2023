package frc.robot.subsystems.intake;

import org.ejml.interfaces.decomposition.CholeskyDecomposition;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Counter;
import frc.robot.Constants.IntakeConstants;;

public class IntakeIOReal implements IntakeIO {
  //  private static final double GEAR_RATIO = 1.5;
  private final Joystick m_joystick = new Joystick(1);
    private boolean isOpen = false;
    private static double kDt = 0.02;
    private static double kP = 1.3;
    private static double kI = 0.0;
    private static double kD = 0.7;


    Counter counter;

    
    private final TalonFX chomp;
  // Note: These gains are fake, and will have to be tuned for your robot.
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);
    private final PIDController chompinPID = new PIDController(kP, kI, kD);  
    private final TrapezoidProfile.Constraints m_constraints =    
    new TrapezoidProfile.Constraints(1.75, 0.75);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State( );
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
        

    public IntakeIOReal() {
       // leftSide = new CANSparkMax(IntakeConstants.intakeLeft, MotorType.kBrushless);
       // rightSide = new CANSparkMax(IntakeConstants.intakeRight, MotorType.kBrushless);
      //  miniVader = new CANSparkMax(IntakeConstants.miniVader, MotorType.kBrushless);
        chomp = new TalonFX(IntakeConstants.intakeChomp);
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

       if (m_joystick.getRawButtonPressed(2)) {
        m_goal = new TrapezoidProfile.State(5, 0);
      } else if (m_joystick.getRawButtonPressed(3)) {
        m_goal = new TrapezoidProfile.State(0, 0);
      }
  
      // Create a motion profile with the given maximum velocity and maximum
      // acceleration constraints for the next setpoint, the desired goal, and the
      // current setpoint.
      var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
  
      // Retrieve the profiled setpoint for the next timestep. This setpoint moves
      // toward the goal while obeying the constraints.
      m_setpoint = profile.calculate(kDt);
  
          // Send setpoint to offboard controller PID
      chomp.setSetpoint(
      chompinPID.
      m_setpoint.position,
      m_feedforward.calculate(m_setpoint.velocity) / 12.0);

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
    
  }

}
