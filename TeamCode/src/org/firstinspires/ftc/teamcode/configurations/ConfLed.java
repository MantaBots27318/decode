/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Limelight configuration data
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.ArrayList;
import java.util.List;

public class ConfLed {

    // To select if the limelight shall be mocked --- not yet activated
    private       boolean                   mShallMock;

    // Mapping between motor name on the hub and the motor direction
    private final List<String>  mHw         = new ArrayList<>();


    public ConfLed(String Name)
    {
        mHw.clear();
        mHw.add(Name);
        mShallMock = false;
    }

    // Constructor for single motor with encoder correction
    public ConfLed(String Name1, String Name2)
    {
        mHw.clear();
        mHw.add(Name1);
        mHw.add(Name2);

        mShallMock    = false;
    }

    public void addHw(String Name) { mHw.add(Name);  }

    public List<String>               getHw()                  { return mHw;}
    public boolean                    shallMock()              { return mShallMock; }

}