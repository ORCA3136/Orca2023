package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOReal implements ElevatorIO{
  private final CANSparkMax elevatorMotor1;
  private final CANSparkMax elevatorMotor2;  
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private SparkMaxPIDController m_pidController1;
  private SparkMaxPIDController m_pidController2;
  private static double kDt = 0.02;
  private double distance;


  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  public ElevatorIOReal() {

    double targetPosition = 0.0;
    elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevator1, MotorType.kBrushless);
    elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevator2, MotorType.kBrushless);

    m_pidController1 = elevatorMotor1.getPIDController(); 
    m_pidController2 = elevatorMotor2.getPIDController(); 


    leftEncoder = elevatorMotor1.getEncoder();
    rightEncoder = elevatorMotor2.getEncoder();

    elevatorMotor1.restoreFactoryDefaults();
    elevatorMotor2.restoreFactoryDefaults();

    elevatorMotor1.enableVoltageCompensation(12.0);
    elevatorMotor2.enableVoltageCompensation(12.0);
    elevatorMotor1.setSmartCurrentLimit(30);
    elevatorMotor2.setSmartCurrentLimit(30);



  }


//    public void teleopPeriodic() {
//    if (m_joystick.getRawButtonPressed(2)) {
//    m_controller.setGoal(5);
//    } else if (m_joystick.getRawButtonPressed(3)) {
//    m_controller.setGoal(0);
//    }
      // Create a PID controller whose setpoint's change is subject to maximum
      // velocity and acceleration constraints.
private final TrapezoidProfile.Constraints m_constraints =
new TrapezoidProfile.Constraints(1.75, 0.75);
private final ProfiledPIDController m_controller =
new ProfiledPIDController(1.3, 0.0, 0.7, m_constraints, kDt);
  
    
public void robotInit() {
  leftEncoder.setPosition(1.0 / 360.0 * 2.0 * Math.PI * 1.5); 
  rightEncoder.setPosition(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
  }

public void teleopPeriodic(){
  // Run controller and update motor output
  elevatorMotor1.set(m_controller.calculate(leftEncoder.getPosition()));
  elevatorMotor2.set(m_controller.calculate(rightEncoder.getPosition()));
}


public void elevatorPower(double elevatorspeed){
  elevatorMotor1.set(elevatorspeed);
  elevatorMotor2.set(elevatorspeed);
}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) 
  {
    inputs.rightPosition = rightEncoder.getPosition();
    inputs.leftPosition = leftEncoder.getPosition();

  }

  @Override
  public double getDistance()
  {
    return distance;
  }

};