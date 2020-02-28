package frc.robot.lib.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
* The Limelight class creates an interface around the network tables that the Limelight uses for communication.
* @see {@link https://docs.limelightvision.io/en/latest/networktables_api.html
*/
public class Limelight {

    // State enumerations
    public static enum LEDMode_t {
        CurrentPipeline { @Override public String toString() { return "Current Pipeline"; } },
        Off { @Override public String toString() { return "Force Off"; } },
        Blink { @Override public String toString() { return "Force Blink"; } },
        On { @Override public String toString() { return "Force On"; } }
    }

    public static enum CameraMode_t { 
        Vision { @Override public String toString() { return "Vision Processor"; } },
        Driver { @Override public String toString() { return "Driver Camera"; } }
    }

    public static enum Pipeline_t { 
        Pipeline0 { @Override public String toString() { return "Pipeline 0"; } },
        Pipeline1 { @Override public String toString() { return "Pipeline 1"; } },
        Pipeline2 { @Override public String toString() { return "Pipeline 2"; } },
        Pipeline3 { @Override public String toString() { return "Pipeline 3"; } },
        Pipeline4 { @Override public String toString() { return "Pipeline 4"; } },
        Pipeline5 { @Override public String toString() { return "Pipeline 5"; } },
        Pipeline6 { @Override public String toString() { return "Pipeline 6"; } },
        Pipeline7 { @Override public String toString() { return "Pipeline 7"; } },
        Pipeline8 { @Override public String toString() { return "Pipeline 8"; } },
        Pipeline9 { @Override public String toString() { return "Pipeline 9"; } }
    }

    public static enum Stream_t { 
        Standard { @Override public String toString() { return "Standard"; } },  // Side-by-side streams if a webcam is attached to Limelight
        PIPMain { @Override public String toString() { return "PIP Main"; } },  // The secondary stream is placed in the lower-right corner of the primary stream
        PIPSecondary { @Override public String toString() { return "PIP Secondary"; } }  // The primary stream is placed in the lower-right corner of the secondary stream
    }

    public static enum Snapshot_t { 
        Stop { @Override public String toString() { return "Stop Snapshots"; } },
        Start { @Override public String toString() { return "Take Snapshots"; } }  // Takes 2 snapshots per second
    }

    // Network table instance copy
    private NetworkTable mTable;


    //-----------------------------------------------------------------------------------------------------------------
    /*                                              PUBLIC API METHODS                                               */
    //-----------------------------------------------------------------------------------------------------------------
    

    public void SetLedMode ( LEDMode_t value ) {
        mTable.getEntry( "ledMode" ).setValue( value.ordinal() );
    }
    
    public double GetLedMode () {
        return mTable.getEntry( "ledMode" ).getDouble( -1.0 );
    }
    
    public void SetCamMode ( CameraMode_t value ) {
        mTable.getEntry( "camMode" ).setValue( value.ordinal() );
    }
    
    public double GetCamMode () {
        return mTable.getEntry("camMode").getDouble( -1.0 );
    }
    
    public void SetPipeline ( Pipeline_t value ) {
        mTable.getEntry( "pipeline" ).setValue( value.ordinal() );
    }
    
    public double GetPipeline () {
        return mTable.getEntry( "pipeline" ).getDouble( -1.0 );
    }
    
    public void SetStream ( Stream_t value ) {
        mTable.getEntry( "stream" ).setValue( value.ordinal() );
    }
    
    public double GetStream () {
        return mTable.getEntry( "stream" ).getDouble( -1.0 );
    }
    
    public void SetSnapshot ( Snapshot_t value ) {
        mTable.getEntry( "snapshot" ).setValue( value.ordinal() );
    }
    
    public double GetSnapshot () {
        return mTable.getEntry( "snapshot" ).getDouble( -1.0 );
    }
    
    public boolean GetFoundTarget () {
        return (mTable.getEntry( "tv" ).getDouble( 0.0 ) == 1.0) ? true : false;
    }
    
    public double GetHorizontalToTargetDeg () {
        return mTable.getEntry( "tx" ).getDouble( 0.0 );
    } 
    
    public double GetVerticalToTargetDeg () {
        return mTable.getEntry( "ty" ).getDouble( 0.0 );
    } 
    
    public double GetTargetAreaPercent () {
        return mTable.getEntry( "ta" ).getDouble( 0.0 );
    } 
    
    public double GetSkewDeg () {
        return mTable.getEntry( "ts" ).getDouble( 0.0 );
    } 
    
    public double GetLatencyMS () {
        return mTable.getEntry( "tl" ).getDouble( 0.0 );
    } 
    
    public double GetShortSidePixels () {
        return mTable.getEntry( "tshort" ).getDouble( 0.0 );
    } 
    
    public double GetLongSidePixels () {
        return mTable.getEntry( "tlong" ).getDouble( 0.0 );
    } 
    
    public double GetHorizontalPixels () {
        return mTable.getEntry( "thor" ).getDouble( 0.0 );
    } 
    
    public double GetVerticalPixels () {
        return mTable.getEntry( "tvert" ).getDouble( 0.0 );
    } 
    
    public double GetActivePipeline () {
        return mTable.getEntry( "getpipe" ).getDouble( 0.0 );
    } 
    
    public double[] GetCameraTransform () {
        final double[] defValue = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        return mTable.getEntry( "camtran" ).getDoubleArray( defValue );
    } 

    public double[] GetCornersX () {
        final double[] defValue = { 0.0, 0.0, 0.0, 0.0 };
        return mTable.getEntry( "tcornx" ).getDoubleArray( defValue );
    }
    
    public double[] GetCornersY () {
        final double[] defValue = { 0.0, 0.0, 0.0, 0.0 };
        return mTable.getEntry( "tcorny" ).getDoubleArray( defValue );
    }

    public double GetRawX0 () {
        return mTable.getEntry( "tx0" ).getDouble( 0.0 );
    } 

    public double GetRawY0 () {
        return mTable.getEntry( "ty0" ).getDouble( 0.0 );
    } 

    public double GetRawArea0 () {
        return mTable.getEntry( "ta0" ).getDouble( 0.0 );
    } 

    public double GetRawSkew0 () {
        return mTable.getEntry( "ts0" ).getDouble( 0.0 );
    } 

    public double GetRawX1 () {
        return mTable.getEntry( "tx1" ).getDouble( 0.0 );
    } 

    public double GetRawY1 () {
        return mTable.getEntry( "ty1" ).getDouble( 0.0 );
    } 

    public double GetRawArea1 () {
        return mTable.getEntry( "ta1" ).getDouble( 0.0 );
    } 

    public double GetRawSkew1 () {
        return mTable.getEntry( "ts1" ).getDouble( 0.0 );
    }     

    public double GetRawX2 () {
        return mTable.getEntry( "tx2" ).getDouble( 0.0 );
    } 

    public double GetRawY2 () {
        return mTable.getEntry( "ty2" ).getDouble( 0.0 );
    } 

    public double GetRawArea2 () {
        return mTable.getEntry( "ta2" ).getDouble( 0.0 );
    } 

    public double GetRawSkew2 () {
        return mTable.getEntry( "ts2" ).getDouble( 0.0 );
    } 

    public double GetCrosshairX0 () {
        return mTable.getEntry( "cx0" ).getDouble( 0.0 );
    } 

    public double GetCrosshairY0 () {
        return mTable.getEntry( "cy0" ).getDouble( 0.0 );
    } 

    public double GetCrosshairX1 () {
        return mTable.getEntry( "cx1" ).getDouble( 0.0 );
    } 

    public double GetCrosshairY1 () {
        return mTable.getEntry( "cy1" ).getDouble( 0.0 );
    }     


    //-----------------------------------------------------------------------------------------------------------------
    /*                                                PRIVATE METHODS                                                */
    //-----------------------------------------------------------------------------------------------------------------


    private void DefaultConfiguration () {
        SetLedMode( LEDMode_t.Off );
        SetCamMode( CameraMode_t.Driver );
        SetPipeline( Pipeline_t.Pipeline0) ;
        SetSnapshot( Snapshot_t.Stop );
        SetStream( Stream_t.Standard );
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                        CLASS CONSTRUCTOR AND OVERRIDES                                        */
    //-----------------------------------------------------------------------------------------------------------------


    public Limelight () {
        mTable = NetworkTableInstance.getDefault().getTable( "limelight" );
        DefaultConfiguration();
    }

    public Limelight ( String tableName ) {
        mTable = NetworkTableInstance.getDefault().getTable( tableName );
        DefaultConfiguration();
    }

}