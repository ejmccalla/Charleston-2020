package frc.robot.lib.drivers;
import edu.wpi.first.wpilibj.DigitalInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* The Photoeye class uses the DigitalInput class to implement the photoeye/beam-break sensors.  This class is
* specifically setup for 5-volt NPN-type sensor outputs and wiring.
*/
public class Photoeye {
    private static final Logger mLogger = LoggerFactory.getLogger( Photoeye.class );
    private final DigitalInput mDigitalInput;

    /**
    * This method will sample the digital input to determine if object has been detected. 
    * @return boolean True if an object is detected, false otherwise
    */    
    public boolean IsPhotoeyeClosed () {
        return mDigitalInput.get();
    }

    /**
    * This is the Photoeye class consructor.
    * @param channel int The digital input channel of the sensor
    * @see {@link edu.wpi.first.wpilibj.DigitalInput}
    */    
    public Photoeye ( int channel ) {
        mDigitalInput = new DigitalInput( channel );
        mLogger.info( "Created photoeye [{}]", channel );
    }

}
