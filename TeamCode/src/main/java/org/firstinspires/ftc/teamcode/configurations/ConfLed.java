/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Limelight configuration data
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

public class ConfLed {

    // To select if the limelight shall be mocked --- not yet activated
    private       boolean                mShallMock;

    // Mapping between limelight name on the hub and the motor direction
    private       String                 mName;

    public ConfLed(String Name)
    {
        mName      = Name;
        mShallMock = false;
    }

    public ConfLed(ConfLed Configuration)
    {
        mShallMock = Configuration.mShallMock;
        mName = Configuration.mName;

    }

    public void addHw(String Name) { mName = Name;  }

    public String                     getHw()                  { return mName;}
    public boolean                    shallMock()              { return mShallMock; }

}