package org.firstinspires.ftc.teamcode.utils;


public class SmartTimer {

    Logger       mLogger;

    private long    mStartTime;
    private boolean mIsRunning;
    private boolean mHasAlreadyBeenCalled;
    private int     mTarget;

    public SmartTimer(Logger logger){
        mIsRunning = false;
        mHasAlreadyBeenCalled = false;
        mStartTime = -10000;
        mLogger = logger;
    }

    public void arm(int milliseconds)
    {
        mStartTime = System.nanoTime();
        mIsRunning = true;
        mTarget = milliseconds;
        mHasAlreadyBeenCalled = true;
    }

    public void reset() {
        mIsRunning = false;
        mHasAlreadyBeenCalled = false;
    }

    public boolean isArmed()
    {
        if(mHasAlreadyBeenCalled) {
            double delta = (System.nanoTime() - mStartTime) / 1_000_000.0;
            if (delta >= mTarget) { mIsRunning = false; }
        }
        return mIsRunning;
    }





}