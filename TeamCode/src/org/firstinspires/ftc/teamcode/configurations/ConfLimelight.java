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

public class ConfLimelight {

    // To select if the limelight shall be mocked --- not yet activated
    private       boolean                mShallMock;

    // Mapping between limelight name on the hub and the motor direction
    private       String                 mName;

    // Reference pipeline identifier for the given limelight
    private final Map<String, Integer>   mPipelines   = new LinkedHashMap<>();

    public ConfLimelight(String Name)
    {
        mName      = Name;
        mShallMock = false;
    }

    public ConfLimelight(ConfLimelight Configuration)
    {
        mShallMock = Configuration.mShallMock;
        mName = Configuration.mName;
        for (Map.Entry<String, Integer> pipeline : Configuration.mPipelines.entrySet()) {
            mPipelines.put(pipeline.getKey(),pipeline.getValue());
        }
    }

    public void addHw(String Name) { mName = Name;  }
    public void addPipeline(String Name, Integer Id)   { mPipelines.put(Name, Id); }

    public String                     getHw()                  { return mName;}
    public boolean                    shallMock()              { return mShallMock; }
    public Map<String, Integer>       getPipelines()           { return mPipelines; }

    public Integer                    getPipeline(String Name) {
        if(mPipelines.containsKey(Name)) {
            return mPipelines.get(Name);
        }
        return -1;
    }

}