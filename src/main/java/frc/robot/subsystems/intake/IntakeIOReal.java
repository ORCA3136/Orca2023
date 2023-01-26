package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class IntakeIOReal implements IntakeIO
{
    private  PneumaticHub m_ph = new PneumaticHub(1);
    //private DoubleSolenoid openSolenoid =m_ph.makeDoubleSolenoid(0, 1);
    private boolean isOpen = false;
    
    private DoubleSolenoid test = new DoubleSolenoid(PneumaticsModuleType.REVPH,0,1);

    public void open()
    {     
        System.out.println(">>OPEN");
        test.toggle();
        isOpen = true;
        m_ph.enableCompressorAnalog(90.0,105.00);
        System.out.println("<<OPEN");
    }

    public void close()
    {
        System.out.println(">>CLOSE");
        test.toggle();
        isOpen = false;
        m_ph.disableCompressor();
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
