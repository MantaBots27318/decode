/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Intake belts subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

/* Configurations includes */
import org.firstinspires.ftc.teamcode.components.LedComponent;
import org.firstinspires.ftc.teamcode.configurations.ConfDistance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.Distance;
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class IntakeBelts {

    Logger                  mLogger;      // Local logger

    boolean                 mReady;       // True if component is able to fulfil its mission
    boolean                 mIsMoving;

    boolean                 mIsReversed;

    MotorComponent          mMotor;       // Motor rotating the wheels
    Distance                mDistance;
    LedComponent            mLed1 = null;
    LedComponent            mLed2 = null;
    // Check if the component is currently moving on command
    public boolean isMoving()   { return mIsMoving;   }

    public boolean isReversed() { return mIsReversed; }

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger, LedComponent led1, LedComponent led2) {
        mLed1 = led1;
        mLed2 = led2;
        mLogger = logger;
        mReady = true;
        mIsMoving = false;
        mIsReversed = false;

        String status = "";

        ConfMotor intake = config.getMotor("intake-belts");
        if(intake == null)  { mReady = false; status += " CONF";}
        else {
            mMotor = MotorComponent.factory(intake,hwm,"intake-belts",logger);
            if (!mMotor.isReady()) { mReady = false; status += " HW";}
        }

        ConfDistance distance = config.getDistance("intake");
        if(distance == null)  { mReady = false; status += " CONF";}
        else {
            mDistance = new Distance(distance,hwm,"intake-distance", logger);
            if (!mDistance.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.info("==>  IN BLT : OK"); }
        else        { logger.warning("==>  IN BLT : KO : " + status); }

    }

    // Start the brushes with a given power
    public void start(double power)   {

        if(mReady)
        {
            mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mMotor.setPower(power);
            mIsMoving = true;
            if(power < 0) { mIsReversed = true; }
            else { mIsReversed = false; }
        }

    }

    // Stop brushes
    public void stop() {
        if(mReady) {
            mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mMotor.setPower(0);
            mIsMoving = false;
            mIsReversed = false;
        }
    }

    // Get the motor current velocity
    public double getVelocity() {

        double result = 0.0;

        if(mReady) { result = mMotor.getVelocity(); }

        return result;
    }

    public double   getDistance() {

        double result = -1;
        if(mReady) {
            result = mDistance.getDistance();
            if((result <= 8) && (mLed1 != null)) { mLed1.blink();  }
            else if (mLed1 != null)              { mLed1.steady(); }
            if((result <= 8) && (mLed2 != null)) { mLed2.blink();  }
            else if (mLed2 != null)              { mLed2.steady(); }
        }
        return result;
    }

    public void persist(Configuration config) {}

}