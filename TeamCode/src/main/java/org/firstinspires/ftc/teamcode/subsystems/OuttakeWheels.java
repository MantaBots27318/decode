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

    private static final int    sTimeOut = 2000; // Timeout in ms

    Logger                      mLogger;      // Local logger

    boolean                     mReady;       // True if component is able to fulfil its mission
    boolean                     mIsMoving;

    SmartTimer                  mTimer;       // Timer for timeout management

    MotorComponent              mMotor;       // Motor rotating the wheels
    double                      mTargetVelocity;
    PIDFCoefficients            mCoefficients;


    // Check if the component is currently moving on command

    public boolean isTransitioning() {
        boolean result = false;
        if(mReady) {
            if (mMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                result = mTimer.isArmed();
            } else if (mMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                double velocity = mMotor.getVelocity();
                mLogger.trace("Velocity : " + velocity);
                double ratio = Math.abs(mTargetVelocity - mMotor.getVelocity());
                mLogger.trace("Difference : " + ratio);
                ratio = ratio / Math.abs(mTargetVelocity);
                mLogger.trace("Ratio : " + ratio);
                result = mTimer.isArmed() && (ratio > 0.01);
            }
        }

        return result;
    }

    public boolean isMoving()        { return mIsMoving;}
    
    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger = logger;
        mReady = true;
        mIsMoving = false;

        mTimer = new SmartTimer(mLogger);

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
                mCoefficients = new PIDFCoefficients(200,3,0,0);
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

    public void control(double velocity)   {

        if(mReady)
        {
            mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //mMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,mCoefficients);
            mMotor.setVelocity(velocity);
            mTargetVelocity = velocity;
            mIsMoving = true;
            mTimer.arm(sTimeOut);
        }

    }

    public void control(double velocity, int timeout) {

        if(mReady && !this.isTransitioning())
        {

            mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //mMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,mCoefficients);
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