package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configurations.ConfDistance;
import org.firstinspires.ftc.teamcode.utils.Logger;


public class Distance {

    Logger              mLogger;

    boolean             mReady;
    String              mName;

    Rev2mDistanceSensor mSensor;

    /* -------------- Constructors --------------- */
    public Distance(ConfDistance conf, HardwareMap hwMap, String name, Logger logger)
    {
        mReady          = true;
        mLogger         = logger;
        mName           = name;

        String hw = conf.getHw();
        mSensor = (Rev2mDistanceSensor)hwMap.tryGet(DistanceSensor.class, hw);
        if(mSensor  == null) { mReady = false; }
        else { mSensor.initialize(); }

    }

    public boolean                      isReady() { return mReady;}
    public String                       getName() { return mName; }

    public double	                    getDistance()
    {
        double result = -1;
        if(mReady) {
            result = mSensor.getDistance(DistanceUnit.INCH);
        }
        return result;
    }

}
