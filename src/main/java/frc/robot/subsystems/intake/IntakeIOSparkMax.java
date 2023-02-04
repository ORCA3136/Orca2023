package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
    private static final double GEAR_RATIO = 1.5;
    
    private final VictorSPX chomp;
    private final CANSparkMax leftSide;
    private final CANSparkMax rightSide;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;


    public IntakeIOSparkMax() {
        leftSide = new CANSparkMax(Constants.intakeLeft, MotorType.kBrushless);
        rightSide = new CANSparkMax(Constants.intakeRight, MotorType.kBrushless);
        chomp = new VictorSPX(Constants.intakeChomp);
        encoder = leftSide.getEncoder();
        pid = leftSide.getPIDController();

        leftSide.restoreFactoryDefaults();
        rightSide.restoreFactoryDefaults();

        rightSide.follow(leftSide);

        leftSide.burnFlash();
        rightSide.burnFlash();
    }

  private void enableVoltageCompensation() {
    }

public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
            * GEAR_RATIO,
        ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }


  public void stop() {
    leftSide.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }

}
