package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class IntakeIOReal implements IntakeIO
{
    private  PneumaticHub m_ph; 
    private DoubleSolenoid openSolenoid; 
    private boolean isOpen = false;
    public IntakeIOReal(){
        m_ph = new PneumaticHub(1);
        m_ph.enableCompressorAnalog(90.0,105.00);
        openSolenoid =m_ph.makeDoubleSolenoid(0, 1);
        openSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void open()
    {     
        System.out.println(">>OPEN");
        openSolenoid.set(DoubleSolenoid.Value.kForward);
        isOpen = true;
        System.out.println("<<OPEN");
    }

    public void close()
    {
        System.out.println(">>CLOSE");
        openSolenoid.set(DoubleSolenoid.Value.kReverse);
        isOpen = false;
        System.out.println("<<CLOSE");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.open = isOpen;
        inputs.pressure= m_ph.getPressure(0); //assumption is we are plugged into 0
        inputs.compressor =  m_ph.getCompressor();

    }


}
