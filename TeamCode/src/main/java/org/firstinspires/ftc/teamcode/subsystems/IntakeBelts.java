/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Intake belts subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Configurations includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

/* Component includes */
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

    // Check if the component is currently moving on command
    public boolean isMoving()   { return mIsMoving;   }

    public boolean isReversed() { return mIsReversed; }

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger = logger;
        mReady = true;
        mIsMoving = false;
        mIsReversed = false;

        String status = "";

        ConfMotor intake = config.getMotor("intake-belts");
        if(intake == null)  { mReady = false; status += " CONF";}
        else {

            // Build motor based on configuration
            if (intake.shallMock()) { mMotor = new MotorMock("intake-belts"); }
            else if (intake.getHw().size() == 1) { mMotor = new MotorSingle(intake, hwm, "intake-belts", logger); }
            else if (intake.getHw().size() == 2) { mMotor = new MotorCoupled(intake, hwm, "intake-belts", logger); }

            if (!mMotor.isReady()) { mReady = false; status += " HW";}
            else {
                // Initialize motor
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

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


    public void persist(Configuration config) {}

}