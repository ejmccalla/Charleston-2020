package frc.robot.lib.drivers;
import edu.wpi.first.wpilibj.DigitalInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Photoeye {
    private static final Logger mLogger = LoggerFactory.getLogger( Photoeye.class );
    private final DigitalInput mDigitalInput;

    public boolean IsPhotoeyeClosed () {
        return mDigitalInput.get();
    }

    public Photoeye(int channel) {
        mDigitalInput = new DigitalInput( channel );
        mLogger.info( "Created photoeye [{}]", channel );
    }

}
