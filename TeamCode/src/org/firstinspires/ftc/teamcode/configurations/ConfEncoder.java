/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Motor configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

public class ConfEncoder {

    // To select if the motor shall be mocked --- not yet activated
    private       boolean               mShallMock;

    // Mapping between motor name on the hub and the motor direction
    private final Map<String, Boolean>  mHw         = new LinkedHashMap<>();
    // Mapping between motor name on the hub and the encoder correction (even when direction is corrected, the encoder may still decrease when power is positive)
    private final Map<String, Boolean>  mDirection    = new LinkedHashMap<>();
    // Reference encoder positions for the given motor (or motor couple)
    private final Map<String, Integer > mPositions  = new LinkedHashMap<>();

    // Constructor for single motor
    public ConfEncoder(String Name, boolean ShallReverse)
    {
        mHw.clear();
        mHw.put(Name,ShallReverse);

        mDirection.clear();
        mDirection.put(Name,false);
        mShallMock    = false;
    }
    // Constructor for coupled motors
    public ConfEncoder(String Name, boolean ShallReverse, boolean ShallInvertEncoder)
    {
        mHw.clear();
        mHw.put(Name,ShallReverse);

        mDirection.clear();
        mDirection.put(Name,ShallInvertEncoder);
        mShallMock    = false;
    }

    // Constructor for single motor with encoder correction
    public ConfEncoder(String Name1, boolean ShallReverse1, String Name2, boolean ShallReverse2)
    {
        mHw.clear();
        mHw.put(Name1,ShallReverse1);
        mHw.put(Name2,ShallReverse2);

        mDirection.clear();
        mDirection.put(Name1,false);
        mDirection.put(Name2,false);
        mShallMock    = false;
    }

    // Constructor for coupled motors with encoder corrections
    public ConfEncoder(String Name1, boolean ShallReverse1, boolean ShallInvertEncoder1, String Name2, boolean ShallReverse2, boolean ShallInvertEncoder2)
    {
        mHw.clear();
        mHw.put(Name1,ShallReverse1);
        mHw.put(Name2,ShallReverse2);

        mDirection.clear();
        mDirection.put(Name1,ShallInvertEncoder1);
        mDirection.put(Name2,ShallInvertEncoder2);
        mShallMock    = false;
    }

    // Basic accessors
    public void addHw(String Name, boolean ShallReverse) { mHw.put(Name,ShallReverse);  }
    public void addPosition(String Name, Integer Value)   { mPositions.put(Name, Value); }

    public Map<String, Boolean>       getHw()                 { return mHw;}
    public Map<String, Boolean>       getDirection()          { return mDirection;}
    public boolean                    shallMock()             { return mShallMock; }
    public Map<String, Integer>       getPositions()          { return mPositions; }

    // Get Motors by index rather than by key ( for tuning )
    public Map.Entry<String, Boolean> getHw(int index)         {
        Map.Entry<String, Boolean> result = null;
        int iHw = 0;
        for (Map.Entry<String, Boolean> pos : mHw.entrySet()) {
            if(iHw == index) { result = pos; }
            iHw ++;
        }
        return result;

    }

    // Get encoders by index rather than by key ( for tuning )
    public Map.Entry<String, Boolean> getEncoder(int index)         {
        Map.Entry<String, Boolean> result = null;
        int iHw = 0;
        for (Map.Entry<String, Boolean> pos : mDirection.entrySet()) {
            if(iHw == index) { result = pos; }
            iHw ++;
        }
        return result;

    }

    // Return target position by its name
    public Integer              getPosition(String Name) {
        if(mPositions.containsKey(Name)) {
            return mPositions.get(Name);
        }
        return -100000;
    }


}