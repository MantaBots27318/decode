/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Intake brushes subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Turret {

    Logger                  mLogger;      // Local logger

    boolean                 mReady;       // True if component is able to fulfil its mission
    boolean                 mIsMoving;

    boolean                 mIsReversed;

    Vision                  mVision;

    MotorComponent          mMotor;       // Motor rotating the wheels

    // Check if the component is currently moving on command
    public boolean isMoving() { return mIsMoving; }

    public boolean isReversed() { return mIsReversed; }

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger     = logger;
        mReady      = true;
        mIsMoving   = false;
        mIsReversed = false;

        String status = "";
        mVision         = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", mLogger);

        ConfMotor turret = config.getMotor("turret");
        if(turret == null)  { mReady = false; status += " CONF";}
        else {

            // Build motor based on configuration
            if (turret.shallMock()) { mMotor = new MotorMock("turret"); }
            else if (turret.getHw().size() == 1) { mMotor = new MotorSingle(turret, hwm, "turret", logger); }
            else if (turret.getHw().size() == 2) { mMotor = new MotorCoupled(turret, hwm, "turret", logger); }

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

    public void loop() {
        Pose3D output = mVision.getPosition();
        while(output == null) { output = mVision.getPosition(); }
        if(output != null){

        }
    }

    // Start the wheels with a given power
    public void start(double power)   {

        if(mReady)
        {
            mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mMotor.setPower(power);
            mIsMoving = true;
            if(power < 0) { mIsReversed = true; }
            else          { mIsReversed = false; }
        }

    }

    // Stop brushes
    public void stop() {
        if(mReady) {
            mMotor.setPower(0);
            mIsMoving = false;
            mIsReversed = false;
        }
    }

    public void persist(Configuration config) {}

    public void faceAT(){
        if (mReady){

        }
    }
}