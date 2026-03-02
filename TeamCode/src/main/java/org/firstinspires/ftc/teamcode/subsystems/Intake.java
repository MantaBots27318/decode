/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Intake brushes subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Intake {

    Logger                  mLogger;      // Local logger
    boolean                 mReady;       // True if component is able to fulfil its mission

    MotorComponent          mIntake;      // Motor rotating the front wheels
    MotorComponent          mGuiding;    // Motor rotating the transfer wheels

    boolean                 mIsStarted;
    boolean                 mIsReversed;


    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger     = logger;
        mReady      = true;
        mIsStarted  = false;

        String status = "";

        ConfMotor intake = config.getMotor("intake-wheels");
        if(intake == null)  { mReady = false; status += " CONF";}
        else {
            mIntake = MotorComponent.factory(intake,hwm,"intake-wheels",logger);
            if (!mIntake.isReady()) { mReady = false; status += " HW";}
        }

        ConfMotor guiding = config.getMotor("guiding-wheels");
        if(guiding == null)  { mReady = false; status += " CONF";}
        else {
            mGuiding = MotorComponent.factory(guiding,hwm,"guiding-wheels",logger);
            if (!mGuiding.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.info("==>  IN BLT : OK"); }
        else        { logger.warning("==>  IN BLT : KO : " + status); }

    }

    public boolean isMoving()   { return mIsStarted; }
    public boolean isReversed() { return mIsReversed; }

    // Start the wheels with a given power
    public void start(double power1,double power2)   {

        if(mReady)
        {
            mIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mIntake.setPower(power1);
            mGuiding.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mGuiding.setPower(power2);
            mIsStarted = true;
            mIsReversed = power1 < 0;
        }

    }

    // Stop brushes
    public void stop() {
        if(mReady) {
            mIntake.setPower(0);
            mGuiding.setPower(0);
            mIsStarted = false;
        }
    }

}