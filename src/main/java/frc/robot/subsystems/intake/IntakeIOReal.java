package frc.robot.subsystems.intake;


public class IntakeIOReal implements IntakeIO
{

    private boolean isOpen = false;
    public IntakeIOReal(){
    
    }

    public void open()
    {     
        System.out.println(">>OPEN");
       
        isOpen = true;
        System.out.println("<<OPEN");
    }

    public void close()
    {
        System.out.println(">>CLOSE");
       
        isOpen = false;
        System.out.println("<<CLOSE");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.open = isOpen;

    }


}
