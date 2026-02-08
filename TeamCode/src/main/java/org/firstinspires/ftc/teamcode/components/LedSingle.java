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

public class LedSingle extends LedComponent {

    Logger              mLogger;

    boolean             mReady;
    String              mName;

    LED                 mRed;
    LED                 mGreen;

    /* -------------- Constructors --------------- */
    public LedSingle(ConfLed conf, HardwareMap hwMap, String name, Logger logger)
    {
        super(logger);

        mReady  = true;
        mLogger = logger;
        mName   = name;

        List<String> hw = conf.getHw();
        if((hw.size() == 1) && !conf.shallMock()) {

            mRed = hwMap.tryGet(LED.class, hw.get(0) + "Red");
            mGreen = hwMap.tryGet(LED.class, hw.get(0) + "Green");
        }

        if(mRed  == null) { mReady = false; }
        if(mGreen  == null) { mReady = false; }

    }

    public boolean isReady() { return mReady; }

    public void on(Color color) {

        if(mReady) {
            mCurrentColor = color;
            if (color == Color.RED) {
                mGreen.off();
                mRed.on();
            }
            if (color == Color.GREEN) {
                mGreen.on();
                mRed.off();
            }
        }
    }

    public void off() {
        if(mReady) {
            mGreen.off();
            mRed.off();
        }
    }
}