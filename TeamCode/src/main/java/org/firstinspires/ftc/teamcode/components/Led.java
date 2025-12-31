/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Led manages REV Leds
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfLed;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Led {

    public enum Color {
        RED,
        GREEN
    }

    Logger              mLogger;

    boolean             mReady;
    String              mName;

    LED                 mRed;
    LED                 mGreen;

    /* -------------- Constructors --------------- */
    public Led(ConfLed conf, HardwareMap hwMap, String name, Logger logger)
    {
        mReady  = true;
        mLogger = logger;
        mName   = name;

        mRed = hwMap.tryGet(LED.class, conf.getHw() + "Red");
        if(mRed  == null) { mReady = false; }

        mGreen = hwMap.tryGet(LED.class, conf.getHw() + "Green");
        if(mGreen  == null) { mReady = false; }

    }

    public void on(Color color) {

        if(mReady) {
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