package org.firstinspires.ftc.teamcode;

/* System includes */

import android.annotation.SuppressLint;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Roadrunner includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

/* Project includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.Poses;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;


public class Driving {

    Telemetry       mLogger;
    Vision          mVision;
    boolean         mReady;

    IMU             mImu;

    double          mHeadingOffset;
    Poses           mPoses;

    MecanumDrive    mDrive;
    SmartTimer      mTimer;
    Action          mAction;

    HardwareMap     mMap;

    MotorComponent  mFrontLeftMotor;
    MotorComponent  mBackLeftMotor;
    MotorComponent  mFrontRightMotor;
    MotorComponent  mBackRightMotor;

    Controller      mGamepad;

    boolean         mIsAutomated = false;
    boolean         mIsFieldCentric = true;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger, Controller gp, Vision vision, Poses poses) {

        mLogger = logger;
        mLogger.addLine("======== DRIVING =========");

        mReady = true;

        mPoses = poses;

        // Get wheels and IMU parameters from configuration
        ConfMotor frontLeftWheel  = config.getMotor("front-left-wheel");
        ConfMotor frontRightWheel = config.getMotor("front-right-wheel");
        ConfMotor backLeftWheel   = config.getMotor("back-left-wheel");
        ConfMotor backRightWheel  = config.getMotor("back-right-wheel");
        ConfImu imu               = config.getImu("built-in");

        mTimer                    = new SmartTimer(mLogger);
        mVision                   = vision;
        mMap = hwm;

        if (mIsFieldCentric) { mLogger.addLine("==>  FIELD CENTRIC"); }
        else                 { mLogger.addLine("==>  ROBOT CENTRIC"); }

        String status = "";
        if(frontLeftWheel == null)         { status += " FL";  mReady = false; }
        if(frontRightWheel == null)        { status += " FR";  mReady = false; }
        if(backLeftWheel == null)          { status += " BL";  mReady = false; }
        if(backRightWheel == null)         { status += " BR";  mReady = false; }
        if(mIsFieldCentric && imu == null) { status += " IMU"; mReady = false; }

        if(mReady) { mLogger.addLine("==>  CONF : OK"); }
        else         { mLogger.addLine("==>  CONF : KO : " + status); }

        if (mReady) {

            status = "";

            mFrontLeftMotor = new MotorSingle(frontLeftWheel, hwm, "front-left-wheel",mLogger);
            mBackLeftMotor = new MotorSingle(backLeftWheel, hwm, "back-left-wheel",mLogger);
            mFrontRightMotor = new MotorSingle(frontRightWheel, hwm, "front-right-wheel",mLogger);
            mBackRightMotor = new MotorSingle(backRightWheel, hwm, "back-right-wheel",mLogger);

            mImu = null;
            if(imu != null) { mImu = hwm.tryGet(IMU.class, imu.getName()); }

            if (!mFrontLeftMotor.isReady())     { status += " FL";  mReady = false; }
            if (!mFrontRightMotor.isReady())    { status += " FR";  mReady = false; }
            if (!mBackLeftMotor.isReady())      { status += " BL";  mReady = false; }
            if (!mBackRightMotor.isReady())     { status += " BR";  mReady = false; }
            if(mIsFieldCentric && mImu == null) { status += " IMU"; mReady = false; }

            if(mReady) { logger.addLine("==>  HW : OK"); }
            else        { logger.addLine("==>  HW : KO : " + status); }

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

            if(mIsFieldCentric) {
                RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                        imu.getLogo(), imu.getUsb());
                mImu.initialize(new IMU.Parameters(RevOrientation));
                mImu.resetYaw();
                mHeadingOffset = 0;
                Double initialHeading = config.retrieve("heading");
                if(initialHeading != null) {
                    // From FTC field reference to initial robot position;
                    mHeadingOffset = initialHeading;
                }
                logger.addLine("==>  Heading Offset : " + mHeadingOffset);
            }

            mDrive                    = new MecanumDrive(hwm, new Pose2d(0,0,0));


        }

        mGamepad = gp;

        if(mReady) {
            mGamepad.axes.left_stick_x.deadzone(0.1);
            mGamepad.axes.right_stick_x.deadzone(0.1);
            mGamepad.axes.left_stick_y.deadzone(0.1);
        }

        if(mReady) { logger.addLine("==>  READY"); }
        else       { logger.addLine("==>  NOT READY"); }
    }


    @SuppressLint("DefaultLocale")
    public void control() {

        if(mReady) {
            if (mGamepad.buttons.a.pressedOnce() && mIsAutomated) {
                mLogger.addLine("==> AUT SHT");
                mIsAutomated = false;
            }
            if (mGamepad.buttons.y.pressedOnce() && !mIsAutomated) {
                mLogger.addLine("==> AUT SHT");
                shootPosition();
            }
        }
        if (mReady && !mIsAutomated) {

            mLogger.addLine("======== DRIVING =========");

            double multiplier = 0.9;
            if (mGamepad.buttons.left_bumper.pressed())  { multiplier = 0.45; }

            double y = mGamepad.axes.left_stick_x.value();
            double x = mGamepad.axes.left_stick_y.value();
            double rotation = mGamepad.axes.right_stick_x.value();
            mLogger.addLine(String.format("\n==>  X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

            if (mIsFieldCentric) {
                double heading = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                heading += mHeadingOffset;
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
                mLogger.addLine(String.format("==>  HD %6.1f X : %6.1f Y : %6.1f", heading,x,y));
            }
            x *= 1.1; // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
            double frontLeftPower = (x + y + rotation) / denominator * multiplier;
            double backLeftPower = (x - y + rotation) / denominator * multiplier;
            double frontRightPower = (x - y - rotation) / denominator * multiplier;
            double backRightPower = (x + y - rotation) / denominator * multiplier;

            mFrontLeftMotor.setPower(frontLeftPower);
            mBackLeftMotor.setPower(backLeftPower);
            mFrontRightMotor.setPower(frontRightPower);
            mBackRightMotor.setPower(backRightPower);
        }
        else if(mIsAutomated && mReady) {

            FtcDashboard.getInstance().getTelemetry().addData("==> AUTOMATED POSE : ", mDrive.getPose());
            FtcDashboard.getInstance().getTelemetry().addData("==> AUTOMATED HEADING : ", (Math.asin(mDrive.getPose().heading.imag) / Math.PI * 180));
            mIsAutomated = mAction.run(new TelemetryPacket());
        }
    }

    public void shootPosition(){

        // Gather April Tags
        Pose3D output = mVision.getPosition();
        mTimer.arm(100);

        while(output == null && mTimer.isArmed()) { output = mVision.getPosition(); }
        if (output != null) {

            Pose2d pose = new Pose2d(
                    -output.getPosition().x * Poses.M_TO_INCHES,
                    -output.getPosition().y * Poses.M_TO_INCHES,
                    (output.getOrientation().getYaw() + 180) * Math.PI / 180);

            mDrive.updatePose(pose);

            mLogger.addLine("==> NEW POSE : " + pose);
            FtcDashboard.getInstance().getTelemetry().addData("==> NEW POSE AT : ", pose);
            FtcDashboard.getInstance().getTelemetry().addData("==> NEW POSE AT HEADING : ", Math.asin(pose.heading.imag) / Math.PI * 180);

            mLogger.addLine("==> NEW POSE : " + mDrive.getPose());
            FtcDashboard.getInstance().getTelemetry().addData("==> NEW POSE : ",mDrive.getPose());
            FtcDashboard.getInstance().getTelemetry().addData("==> NEW POSE HEADING : ",Math.asin(mDrive.getPose().heading.imag) / Math.PI * 180);

            mLogger.addData("==> X ",mPoses.posShootingFTCInches().x);
            mLogger.addData("==> Y ",mPoses.posShootingFTCInches().y);
            FtcDashboard.getInstance().getTelemetry().addLine("==> X : " + mPoses.posShootingFTCInches().x);
            FtcDashboard.getInstance().getTelemetry().addLine("==> Y : " + mPoses.posShootingFTCInches().y);

            mDrive                    = new MecanumDrive(mMap, pose);

            mAction = mDrive.actionBuilder(mDrive.getPose())
                    .splineTo(mPoses.posShootingFTCInches(), mPoses.hShootingFTCRadians())
                    .build();

            mIsAutomated = mAction.run(new TelemetryPacket());

        }


    }

}