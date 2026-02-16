/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   V2 robot commands
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.LedComponent;
import org.firstinspires.ftc.teamcode.components.LedCoupled;
import org.firstinspires.ftc.teamcode.components.LedMock;
import org.firstinspires.ftc.teamcode.components.LedSingle;
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.ConfLed;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.LockQRCode;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBelts;
import org.firstinspires.ftc.teamcode.subsystems.IntakeEntryArm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeLeverArm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWheels;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.utils.VelocityAbacus;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class RobotFebEight {

    enum Mode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        QRCODE_CENTRIC
    }

    Logger                  mLogger;
    boolean                 mReady;
    SmartTimer              mTimer;

    IntakeBelts             mIntakeBelts;

    double                  mTargetVelocity;
    double                  mTargetDistance;

    Chassis                 mChassis;
    IMU                     mImu;
    double                  mHeadingOffset;
    Path                    mPath;
    LockQRCode              mLocker;

    double                  mX;
    double                  mY;
    double                  mRotation;

    Vision                  mVision;

    Controller              mGamepadChassis;
    Controller              mGamepadAttachments;

    Mode                    mMode = Mode.FIELD_CENTRIC;
    Mode                    mFallbackMode;

    boolean                 mIsPrecise = false;
    boolean                 mIsMoving;

    Pose2d                  mPark;
    Action                  mAction;

    boolean                 mShallCorrectSmallResidue = false;


    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Controller gamepad1, Controller gamepad2, Path path) {

        mLogger = logger;

        mReady = true;

        if(mReady) {
            mTimer              = new SmartTimer(mLogger);
            mGamepadChassis     = gamepad1;
            mGamepadAttachments = gamepad2;
            mPath               = path;
            mFallbackMode       = mMode;
        }

        if(mReady) { mReady = this.initialize_collecting(config, hwm); }
        if(mReady) { mReady = this.initialize_drive(config, hwm);      }


        if(mReady) {
            mGamepadChassis.axes.left_stick_x.deadzone(0.1);
            mGamepadChassis.axes.right_stick_x.deadzone(0.1);
            mGamepadChassis.axes.left_stick_y.deadzone(0.1);
            mIsMoving = false;
        }

        if(mReady) { mLogger.info("==>  READY"); }
        else       { mLogger.warning("==>  NOT READY"); }
    }


    public void close()
    {
        if(mReady) { mVision.close(); }
    }

    public void control() {

        control_chassis();
        control_attachments();

        mY = mGamepadChassis.axes.left_stick_x.value();
        mX = mGamepadChassis.axes.left_stick_y.value();
        mRotation = mGamepadChassis.axes.right_stick_x.value();

        if (mReady) {

            mLogger.info("======== DRIVING =========");
            if (mMode == Mode.FIELD_CENTRIC)       { mLogger.info("==> MODE : FC"); }
            else if (mMode == Mode.ROBOT_CENTRIC)  { mLogger.info("==> MODE : RC"); }
            else if (mMode == Mode.QRCODE_CENTRIC && !mShallCorrectSmallResidue) { mLogger.info("==> MODE : QC"); }
            else if (mMode == Mode.QRCODE_CENTRIC && mShallCorrectSmallResidue) { mLogger.info("==> MODE : QC CORRECT"); }

            mLogger.info("======= COLLECTING =======");
            mLogger.info("==> SHOOTING NOT ENGAGED");
        }

    }

    public void loop() {

        mIsMoving = (Math.sqrt(mX * mX + mY * mY) >= 0.01);


        move(mX, mY, mRotation);

    }

    void control_chassis() {

        if (mGamepadChassis.buttons.right_trigger.pressed()) {
            mLogger.info("==> QRCODE MODE RIGHT TRIGGER");
            mMode = Mode.QRCODE_CENTRIC;
            mShallCorrectSmallResidue = false;
        }
        else if (mGamepadChassis.buttons.right_bumper.pressed()) {
            mLogger.info("==> QRCODE MODE RIGHT BUMPER");
            mMode = Mode.QRCODE_CENTRIC;
            mShallCorrectSmallResidue = true;
        }
        else if (mGamepadChassis.buttons.right_trigger.notPressed() && mGamepadChassis.buttons.right_bumper.notPressed()) {
            mLogger.info("==> FALLBACK MODE");
            mMode = mFallbackMode;
            mShallCorrectSmallResidue = false;
        }
        if (mGamepadChassis.buttons.a.pressedOnce()) {
            if(mFallbackMode == Mode.FIELD_CENTRIC)      {
                mLogger.info("==> ROBOT CENTRIC MODE");
                mFallbackMode = mMode = Mode.ROBOT_CENTRIC;
                mShallCorrectSmallResidue = false;
            }
            else if(mFallbackMode == Mode.ROBOT_CENTRIC)      {
                mLogger.info("==> FIELD CENTRIC MODE");
                mFallbackMode = mMode = Mode.FIELD_CENTRIC;
                mShallCorrectSmallResidue = false;
            }
        }


        mIsPrecise = mGamepadChassis.buttons.left_bumper.pressed();

    }

    void control_attachments() {

        if (mGamepadChassis.buttons.a.pressedOnce())   { start_intake();   }
        if (mGamepadChassis.buttons.b.pressedOnce())   { stop_intake();   }
    }

    void move(double x, double y, double rotation) {

        double             multiplier = 0.9;
        if (mIsPrecise)  { multiplier = 0.3; }

        mLogger.info(String.format("==>  X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

        if(mMode == Mode.ROBOT_CENTRIC) {
            mChassis.drive(x,y,rotation, 0, multiplier);
        }
        else if (mMode == Mode.FIELD_CENTRIC)
        {

            double heading = mLocker.getPosition().heading.toDouble() - mHeadingOffset;
            mLogger.trace("yaw : " + heading / Math.PI * 180);
            mChassis.drive(x,y,rotation, heading, multiplier);
        }
        else if(mMode == Mode.QRCODE_CENTRIC) {
            if(mLocker.isSet()) {

                if(!mIsMoving && !mShallCorrectSmallResidue) {
                    mChassis.drive(x, y, 0, mLocker.getHeading(), multiplier);
                }
                else if (!mShallCorrectSmallResidue){
                    mChassis.drive(x, y, mLocker.getRotation1(), mLocker.getHeading(), multiplier);
                }
                else {
                    mChassis.drive(x, y, mLocker.getRotation2(), mLocker.getHeading(), multiplier);
                }
            }
        }
    }

    boolean initialize_collecting(Configuration config, HardwareMap hwm) {

        boolean result = true;

        mLogger.info("======= COLLECTING =======");

        mIntakeBelts        = new IntakeBelts();

        mIntakeBelts.setHW(config, hwm, mLogger,null, null);

        return result;

    }

    boolean initialize_drive(Configuration config, HardwareMap hwm) {

        boolean result = true;

        mLogger.info("======== DRIVING =========");

        // Get wheels and IMU parameters from configuration
        ConfImu imu = config.getImu("built-in");

        if (mMode == Mode.FIELD_CENTRIC)      { mLogger.info("==>  FIELD CENTRIC"); }
        else if (mMode == Mode.ROBOT_CENTRIC) { mLogger.info("==>  ROBOT CENTRIC"); }

        String status = "";
        if (imu == null)             { status += " IMU"; result = false; }

        if (result) {

            status = "";

            mChassis = new Chassis();
            mChassis.setHW(config, hwm, mLogger);

            mImu = null;
            mImu = hwm.tryGet(IMU.class, imu.getName());
            if (mImu == null)                { status += " IMU"; result = false; }
        }

        if (result) {

            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    imu.getLogo(), imu.getUsb());
            mImu.initialize(new IMU.Parameters(RevOrientation));
            mImu.resetYaw();
            double heading = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            mLogger.trace("yaw : " + heading / Math.PI * 180);

            mHeadingOffset = mPath.fieldCentric2FTC();

        }

        if(result) { mLogger.info("==>  CONF : OK"); }
        else       { mLogger.warning("==>  CONF : KO : " + status); }


        return result;

    }

    public boolean start_intake() {
        mLogger.info("==> STR INTAKE");
        mIntakeBelts.start(-1.0);
        return false;
    }
    public boolean reverse_intake() {
        mLogger.info("==> RVS INTAKE");
        mIntakeBelts.start(1.0);
        return false;
    }
    public boolean stop_intake() {
        mLogger.info("==> STP INTAKE");
        mIntakeBelts.stop();
        return false;
    }

}