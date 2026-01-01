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

import org.firstinspires.ftc.teamcode.configurations.ConfLed;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class LedMock extends LedComponent {


    String              mName;

    Color               mColor;

    /* -------------- Constructors --------------- */
    public LedMock(String name)
    {
        mName   = name;
        mColor  = Color.NONE;
    }

    public boolean isReady() { return true; }

    public void on(Color color) {
        mColor = color;
    }

    public void off() {
        mColor = Color.NONE;
    }
}