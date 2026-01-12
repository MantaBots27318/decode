/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Outtake wheels subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class OuttakeWheels {

    private static final int    sTimeOut = 10000; // Timeout in ms

    Logger                      mLogger;         // Local logger

    boolean                     mReady;          // True if component is able to fulfil its mission
    boolean                     mIsMoving;
    boolean                     mIsWaiting;

    SmartTimer                  mTimer;       // Timer for timeout management
    SmartTimer                  mTimerShoot;  // Timer for timeout management

    MotorComponent              mMotor;       // Motor rotating the wheels
    double                      mTargetVelocity;
    PIDFCoefficients            mCoefficients;


    // Check if the component is currently moving on command

    public boolean isTransitioning() {
        boolean result = true;
        if(mReady) {
            if (mMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                result = mTimer.isArmed();
            } else if (mMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                double velocity =  mMotor.getVelocity();
                double ratio = Math.abs(mTargetVelocity - velocity);
                mLogger.trace("current vel : " +velocity);
                mLogger.trace("target vel : "+mTargetVelocity);
                mLogger.trace("ratio : " + ratio);
                mLogger.trace("timeout : " + mTimer.isArmed());
                if(!mIsWaiting && (ratio < 0.05)) {
                    mTimerShoot.arm(500);
                    mIsWaiting = true;
                }
                mLogger.trace("timer : " + mTimerShoot.isArmed());
                mLogger.trace("is waiting : " + mIsWaiting);
                result = mTimer.isArmed() && (mTimerShoot.isArmed() || !mIsWaiting);
                mLogger.trace("result : " +result);
                mLogger.update();
                if(!result) { mIsWaiting = false; }
            }
        }

        return result;
    }

    public void stopTransition() {
        mIsWaiting = false;

        mTimer.reset();
        mTimerShoot.reset();
    }

    public boolean isMoving()        { return mIsMoving;}
    
    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger = logger;
        mReady = true;
        mIsMoving = false;
        mIsWaiting = false;

        mTimer = new SmartTimer(mLogger);
        mTimerShoot = new SmartTimer(mLogger);

        String status = "";

        ConfMotor wheels = config.getMotor("outtake-wheels");
        if(wheels == null)  { mReady = false; status += " CONF";}
        else {

            // Build motor based on configuration
            if (wheels.shallMock()) { mMotor = new MotorMock("outtake-wheels"); }
            else if (wheels.getHw().size() == 1) { mMotor = new MotorSingle(wheels, hwm, "outtake-wheels", logger); }
            else if (wheels.getHw().size() == 2) { mMotor = new MotorCoupled(wheels, hwm, "outtake-wheels", logger); }

            if (!mMotor.isReady()) { mReady = false; status += " HW";}
            else {
                // Initialize motor
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                mCoefficients = new PIDFCoefficients(300,50,100,0);
                mMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,mCoefficients);
            }

        }

        // Log status
        if (mReady) { logger.info("==>  OUT WHEELS : OK"); }
        else        { logger.warning("==>  OUT WHEELS : KO : " + status); }

    }

    // Get the motor current velocity
    public double getVelocity() {

        double result = 0.0;

        if(mReady) { result = mMotor.getVelocity(); }

        return result;
    }

    public void control(double velocity, boolean ShallSetTimeOut)   {

        if(mReady)
        {
            mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,mCoefficients);
            mMotor.setVelocity(velocity);
            mTargetVelocity = velocity;
            mIsMoving = true;
            if(ShallSetTimeOut) { mTimer.arm(sTimeOut); }
        }

    }

    public void control(double velocity, int timeout) {

        if(mReady)
        {

            mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,mCoefficients);
            mMotor.setVelocity(velocity);
            mTargetVelocity = velocity;
            mIsMoving = true;
            mTimer.arm(timeout);
        }

    }

    // Stop brushes
    public void stop() {
        if(mReady) {
            mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mMotor.setPower(0);
            mIsMoving = false;
        }
    }

    public void persist(Configuration config) {}

}