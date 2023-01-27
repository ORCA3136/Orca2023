package frc.robot.subsystems.limeLight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase 
{
    private NetworkTable limelightTable;
    private NetworkTableEntry pipeline;
    private NetworkTableEntry ledMode;
    private boolean alignedToTarget = false;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private double Kp = -0.1; 
    private boolean done = false;

    private static int LED_OFF = 1;
    private static int LED_ON= 2;
    private static int RING = 0;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    private final LimelightIO io;
 


    public void enableLED()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void disableLED()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void findRing()
    {
        pipeline.setNumber(RING);
    }
    
    public Limelight(LimelightIO io) {
        this.io = io;
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        pipeline = limelightTable.getEntry("pipeline");
    }

    @Override
    public void periodic() {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Intake", inputs);
  
    }



    public OnTarget aim() {
        //read values periodically
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
    
        OnTarget target = new OnTarget(); 
  
  
        double steeringAdjust = Kp * x;

        double left=steeringAdjust;
        double right=-steeringAdjust;

  
  


        return target;
  
      }
      
      
  
  }

