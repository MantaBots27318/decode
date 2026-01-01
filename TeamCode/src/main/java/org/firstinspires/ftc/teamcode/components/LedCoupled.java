/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Led manages REV Leds
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* System includes */
import java.util.List;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfLed;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class LedCoupled extends LedComponent{

    Logger              mLogger;

    boolean             mReady;
    String              mName;

    LED                 mFirstRed;
    LED                 mFirstGreen;
    LED                 mSecondRed;
    LED                 mSecondGreen;

    /* -------------- Constructors --------------- */
    public LedCoupled(ConfLed conf, HardwareMap hwMap, String name, Logger logger)
    {
        mReady  = true;
        mLogger = logger;
        mName   = name;

        List<String> hw = conf.getHw();
        if((hw.size() == 2) && !conf.shallMock()) {

            mFirstRed = hwMap.tryGet(LED.class, hw.get(0)+ "Red");
            mFirstGreen = hwMap.tryGet(LED.class, hw.get(0) + "Green");

            mSecondRed = hwMap.tryGet(LED.class, hw.get(1)+ "Red");
            mSecondGreen = hwMap.tryGet(LED.class, hw.get(1) + "Green");
        }

        if(mFirstRed  == null)   { mReady = false; }
        if(mFirstGreen == null)  { mReady = false; }
        if(mSecondRed  == null)  { mReady = false; }
        if(mSecondGreen == null) { mReady = false; }

    }

    public boolean isReady() { return mReady; }

    public void on(Color color) {

        if(mReady) {
            if (color == Color.RED) {
                mFirstGreen.off();
                mSecondGreen.off();
                mFirstRed.on();
                mSecondRed.on();
            }
            if (color == Color.GREEN) {
                mFirstGreen.on();
                mSecondGreen.on();
                mFirstRed.off();
                mSecondRed.off();
            }
        }
    }

    public void off() {
        if(mReady) {
            mFirstGreen.off();
            mSecondGreen.off();
            mFirstRed.off();
            mSecondRed.off();
        }
    }
}