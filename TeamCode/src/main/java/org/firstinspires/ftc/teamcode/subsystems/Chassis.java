/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Chassis subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;


/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* Acmerobotics includes */
import com.acmerobotics.roadrunner.Pose2d;

/* Pose includes */
import org.firstinspires.ftc.teamcode.pose.Posable;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Utils includes */
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Chassis implements Posable {

    Logger mLogger;
    boolean mReady;

    MotorComponent mFrontLeftMotor;
    MotorComponent mBackLeftMotor;
    MotorComponent mFrontRightMotor;
    MotorComponent mBackRightMotor;

    PinpointLocalizer mLocalizer;

    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger = logger;
        mLogger.info(Logger.Target.DRIVER_STATION, "======== DRIVING =========");

        mLogger = logger;
        mReady = true;

        String status = "";

        // Get wheels and IMU parameters from configuration
        ConfMotor frontLeftWheel = config.getMotor("front-left-wheel");
        ConfMotor frontRightWheel = config.getMotor("front-right-wheel");
        ConfMotor backLeftWheel = config.getMotor("back-left-wheel");
        ConfMotor backRightWheel = config.getMotor("back-right-wheel");
        ConfImu pinpoint = config.getImu("pinpoint");

        if (frontLeftWheel == null) { status += " FL"; mReady = false; }
        if (frontRightWheel == null) { status += " FR"; mReady = false;}
        if (backLeftWheel == null) { status += " BL"; mReady = false; }
        if (backRightWheel == null) { status += " BR"; mReady = false; }
        if (pinpoint == null) { status += " PPT";  mReady = false; }

        if (mReady) {

            mFrontLeftMotor  = new MotorSingle(frontLeftWheel, hwm, "front-left-wheel", mLogger);
            mBackLeftMotor   = new MotorSingle(backLeftWheel, hwm, "back-left-wheel", mLogger);
            mFrontRightMotor = new MotorSingle(frontRightWheel, hwm, "front-right-wheel", mLogger);
            mBackRightMotor  = new MotorSingle(backRightWheel, hwm, "back-right-wheel", mLogger);
            mLocalizer       = new PinpointLocalizer(hwm, pinpoint.getName(), MecanumDrive.PARAMS.inPerTick, pinpoint.getParReversed(), pinpoint.getPerpReversed(), new Pose2d(0,0,0));

            if (!mFrontLeftMotor.isReady()) { status += " FL"; mReady = false; }
            if (!mFrontRightMotor.isReady()) { status += " FR"; mReady = false; }
            if (!mBackLeftMotor.isReady()) { status += " BL"; mReady = false; }
            if (!mBackRightMotor.isReady()) { status += " BR"; mReady = false; }

        }

        if (mReady) {

            mFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            PinpointLocalizer.PARAMS.parYTicks = pinpoint.getParY() / MecanumDrive.PARAMS.inPerTick;
            PinpointLocalizer.PARAMS.perpXTicks = pinpoint.getPerpX() / MecanumDrive.PARAMS.inPerTick;
            Double initial_heading = config.retrieve("heading");
            if (initial_heading == null) { initial_heading = 0.0; }
            Double initial_x = config.retrieve("x");
            if (initial_x == null) { initial_x = 0.0; }
            Double initial_y = config.retrieve("y");
            if (initial_y == null) { initial_y = 0.0; }
            mLocalizer.setPose(new Pose2d(initial_x,initial_y,initial_heading));

        }

        // Log status
        if (mReady) { mLogger.info("==>  CHS : OK"); }
        else { mLogger.warning("==>  CHS : KO : " + status); }

    }

    public void drive(double x, double y, double rotation, double heading, double multiplier) {

        if (mReady) {

            mLocalizer.update();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotY = x * Math.sin(heading) + y * Math.cos(heading);
            rotX *= 1.1; // Counteract imperfect strafing

            mLogger.info(String.format("==>  ROT: %2.2f HD : %6.1f X : %6.1f Y : %6.1f", rotation / Math.PI * 180, heading / Math.PI * 180, rotX, rotY));

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
            double frontLeftPower = (rotX + rotY + rotation) / denominator * multiplier;
            double backLeftPower = (rotX - rotY + rotation) / denominator * multiplier;
            double frontRightPower = (rotX - rotY - rotation) / denominator * multiplier;
            double backRightPower = (rotX + rotY - rotation) / denominator * multiplier;

            mLogger.trace(Logger.Target.FILE, "before motor powering");
            mFrontLeftMotor.setPower(frontLeftPower);
            mBackLeftMotor.setPower(backLeftPower);
            mFrontRightMotor.setPower(frontRightPower);
            mBackRightMotor.setPower(backRightPower);
            mLogger.trace(Logger.Target.FILE, "after motor powering");
        }
    }

    @Override
    public void setFTCPosition(Pose2d ftcPosition) {
       if(mReady) {
           mLocalizer.setPose(ftcPosition);
       }
    }

    @Override
    public Pose2d getFTCPosition() {
        return mLocalizer.getPose();
    }

    public double getXVelocity()
    {
        double result = 0;
        if(mReady) {
            mLocalizer.driver.getVelX(DistanceUnit.INCH);
        }
        return result;
    }

    public double getYVelocity()
    {
        double result = 0;
        if(mReady) {
            mLocalizer.driver.getVelY(DistanceUnit.INCH);
        }
        return result;
    }

}
