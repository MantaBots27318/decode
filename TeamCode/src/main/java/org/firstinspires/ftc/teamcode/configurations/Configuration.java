/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Generic configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

abstract public class Configuration {

    public static final double FIELD_SIZE_INCHES              = 12 * 12;
    public static final double X_SHOOTING_FTC_INCHES          = 24;
    public static final double Y_SHOOTING_FTC_INCHES          = 24;
    public static final double ANGLE_SHOOTING_FTC_RADIANS     = Math.PI / 4;

    public static final double M_TO_INCHES                    = 39.37;


    // Map to store hardware components by reference name
    protected final  Map<String, ConfMotor>     mMotors         = new LinkedHashMap<>();
    protected final  Map<String, ConfImu>       mImus           = new LinkedHashMap<>();
    protected final  Map<String, ConfServo>     mServos         = new LinkedHashMap<>();
    protected final  Map<String, ConfLimelight> mLimelights     = new LinkedHashMap<>();
    protected        Map<String, Double>        mInterOpModes   = new LinkedHashMap<>();

    // Current selected configuration
    public static Configuration s_Current = new LastYear();

    // Method to retrieve a motor by its reference name
    public ConfMotor getMotor(String name) {
        return mMotors.getOrDefault(name, null);
    }

    // Method to retrieve an imu by its reference name
    public ConfImu getImu(String name) {
        return mImus.getOrDefault(name, null);
    }

    // Method to retrieve a pipeline by its reference name
    public ConfLimelight getLimelight(String name) { return mLimelights.getOrDefault(name, null);}

    // Method to retrieve a servo by its reference name
    public ConfServo getServo(String name) {
        return mServos.getOrDefault(name, null);
    }

    // Method to retrieve all servos uncoupled for tuning
    public Map<String, ConfServo>   getForTuning() { return mServos; }

    // Abstract method for initializing specific configurations
    protected abstract void initialize();

    // Constructor
    public Configuration() {
        initialize();
    }
    
    public void reinit() {
        mInterOpModes.clear();
    }

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
