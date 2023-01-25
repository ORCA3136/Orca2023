package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class IntakeIOReal implements IntakeIO
{
    private  PneumaticHub m_ph = new PneumaticHub(1);
    private DoubleSolenoid openSolenoid =m_ph.makeDoubleSolenoid(14, 15);
    private boolean isOpen = false;
 

    public void open()
    {
        openSolenoid.toggle();
        isOpen = true;
    }

    public void close()
    {
        openSolenoid.toggle();
        isOpen = false;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.open = isOpen;
        inputs.pressure= m_ph.getPressure(0); //assumption is we are plugged into 0
        inputs.compressor =  m_ph.getCompressor();

    }


}
