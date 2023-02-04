package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
    private static final double GEAR_RATIO = 1.5;

    private final CANSparkMax leader;
    private final CANSparkMax follower;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    public IntakeIOSparkMax() {
        leader = new CANSparkMax(1, MotorType.kBrushless);
        follower = new CANSparkMax(2, MotorType.kBrushless);

        encoder = leader.getEncoder();
        pid = leader.getPIDController();

        leader.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();

        leader.burnFlash();
        follower.burnFlash();
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
    leader.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }

}
