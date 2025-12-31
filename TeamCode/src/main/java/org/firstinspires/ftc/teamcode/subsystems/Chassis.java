/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Chassis subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* System includes */
import android.annotation.SuppressLint;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Chassis {

    Logger          mLogger;
    boolean         mReady;

    IMU             mImu;

    MotorComponent  mFrontLeftMotor;
    MotorComponent  mBackLeftMotor;
    MotorComponent  mFrontRightMotor;
    MotorComponent  mBackRightMotor;

    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger = logger;
        mLogger.info(Logger.Target.DRIVER_STATION,"======== DRIVING =========");

        mLogger = logger;
        mReady  = true;

        String  status = "";

        // Get wheels and IMU parameters from configuration
        ConfMotor frontLeftWheel  = config.getMotor("front-left-wheel");
        ConfMotor frontRightWheel = config.getMotor("front-right-wheel");
        ConfMotor backLeftWheel   = config.getMotor("back-left-wheel");
        ConfMotor backRightWheel  = config.getMotor("back-right-wheel");
        ConfImu   imu             = config.getImu("built-in");

        if(frontLeftWheel == null)  { status += " FL";  mReady = false; }
        if(frontRightWheel == null) { status += " FR";  mReady = false; }
        if(backLeftWheel == null)   { status += " BL";  mReady = false; }
        if(backRightWheel == null)  { status += " BR";  mReady = false; }
        if(imu == null)             { status += " IMU"; mReady = false; }

        if (mReady) {

            mFrontLeftMotor = new MotorSingle(frontLeftWheel, hwm, "front-left-wheel",mLogger);
            mBackLeftMotor = new MotorSingle(backLeftWheel, hwm, "back-left-wheel",mLogger);
            mFrontRightMotor = new MotorSingle(frontRightWheel, hwm, "front-right-wheel",mLogger);
            mBackRightMotor = new MotorSingle(backRightWheel, hwm, "back-right-wheel",mLogger);

            if (!mFrontLeftMotor.isReady())     { status += " FL";  mReady = false; }
            if (!mFrontRightMotor.isReady())    { status += " FR";  mReady = false; }
            if (!mBackLeftMotor.isReady())      { status += " BL";  mReady = false; }
            if (!mBackRightMotor.isReady())     { status += " BR";  mReady = false; }

        }

        if(mReady) {

            mFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        // Log status
        if (mReady) { mLogger.info("==>  CHS : OK"); }
        else        { mLogger.warning("==>  CHS : KO : " + status); }

    }


    @SuppressLint("DefaultLocale")
    public void drive( double x, double y, double rotation, double heading, double multiplier) {

        if(mReady) {

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-heading) + y * Math.sin(-heading);
            double rotY = - x * Math.sin(-heading) + y * Math.cos(-heading);
            rotX *= 1.1; // Counteract imperfect strafing

            mLogger.info(Logger.Target.DRIVER_STATION,String.format("==>  ROT: %2.2f HD : %6.1f X : %6.1f Y : %6.1f",rotation,heading /Math.PI * 180,rotX,rotY));

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
            double frontLeftPower = (rotX + rotY + rotation) / denominator * multiplier;
            double backLeftPower = (rotX - rotY + rotation) / denominator * multiplier;
            double frontRightPower = (rotX - rotY - rotation) / denominator * multiplier;
            double backRightPower = (rotX + rotY - rotation) / denominator * multiplier;

            mFrontLeftMotor.setPower(frontLeftPower);
            mBackLeftMotor.setPower(backLeftPower);
            mFrontRightMotor.setPower(frontRightPower);
            mBackRightMotor.setPower(backRightPower);
        }
    }
}
