/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Generic configuration to switch between robot versions
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

abstract public class Configuration {

    public enum Version {
        V1,
        V2,
        NONE
    }

    protected        Version                     mVersion = Version.NONE;

    // Map to store hardware components by reference name
    protected final  Map<String, ConfMotor>     mMotors         = new LinkedHashMap<>();
    protected final  Map<String, ConfImu>       mImus           = new LinkedHashMap<>();
    protected final  Map<String, ConfServo>     mServos         = new LinkedHashMap<>();
    protected final  Map<String, ConfLimelight> mLimelights     = new LinkedHashMap<>();
    protected final  Map<String, ConfLed>       mLeds           = new LinkedHashMap<>();
    protected final  Map<String, ConfDistance>  mDistances      = new LinkedHashMap<>();
    protected        Map<String, Double>        mInterOpModes   = new LinkedHashMap<>();

    // Current selected configuration
    public static Configuration s_Current = new V2();

    // Method to retrieve configuration version
    public Version          getVersion()                { return mVersion; }

    // Method to retrieve a motor by its reference name
    public ConfMotor        getMotor(String name)       { return mMotors.getOrDefault(name, null); }

    // Method to retrieve an imu by its reference name
    public ConfImu          getImu(String name)         { return mImus.getOrDefault(name, null); }

    // Method to retrieve a limelight by its reference name
    public ConfLimelight    getLimelight(String name)   { return mLimelights.getOrDefault(name, null);}

    // Method to retrieve a servo by its reference name
    public ConfServo        getServo(String name)       { return mServos.getOrDefault(name, null);  }

    // Method to retrieve a led by its reference name
    public ConfLed          getLed(String name)         { return mLeds.getOrDefault(name, null);  }

    // Method to retrieve a led by its reference name
    public ConfDistance     getDistance(String name)    { return mDistances.getOrDefault(name, null);  }

    // Method to retrieve all servos uncoupled for tuning
    public Map<String, ConfServo>   getForTuning()      { return mServos; }

    // Abstract method for initializing specific configurations
    protected abstract void initialize();

    // Constructor
    public Configuration() {
        initialize();
    }
    
    public void reinit() { mInterOpModes.clear(); }

    public void persist(String key, double data) {
        mInterOpModes.put(key, data);
    }

    public Double retrieve(String key) {
        Double result = null;
        if(mInterOpModes.containsKey(key)) {
            result = mInterOpModes.get(key);
        }
        return result;
    }
}
