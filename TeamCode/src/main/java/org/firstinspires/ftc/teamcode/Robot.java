/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   V2 robot commands
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/* Acmerobotics includes */
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/* Ftc Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.Controller;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Path includes */
import org.firstinspires.ftc.teamcode.pose.LockQRCode;
import org.firstinspires.ftc.teamcode.pose.Path;

/* Roadrunner includes */
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/* Subsystem includes */
import org.firstinspires.ftc.teamcode.subsystems.IntakeBelts;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeLeverArm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.utils.VelocityAbacus;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {

    public enum Engage {
        NONE,
        WAITING,
        WHEELS
    }

    public enum Shoot {
        NONE,
        WAITING,
        ARM
    }

    enum Mode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        QRCODE_CENTRIC
    }

    Logger                  mLogger;
    boolean                 mReady;
    SmartTimer              mTimer;

    IntakeBelts             mIntakeBelts;
    OuttakeWheels           mOuttakeWheels;
    OuttakeLeverArm         mOuttakeLeverArm;
    double                  mEngageVelocity;

    Camera                  mCamera;

    Chassis                 mChassis;
    IMU                     mImu;
    MecanumDrive            mDrive;
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
    Engage                  mEngageMode;
    Shoot                   mShootMode;
    boolean                 mIsEngaged;


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
        if(mReady) { mReady = this.initialize_vision(config, hwm);     }
        if(mReady) { mReady = this.initialize_drive(config, hwm);      }
        if(mReady) {
            mLocker = new LockQRCode();
            mLocker.setHW(config,hwm,mLogger,path,mVision);
        }

        if(mReady) {
            mGamepadChassis.axes.left_stick_x.deadzone(0.1);
            mGamepadChassis.axes.right_stick_x.deadzone(0.1);
            mGamepadChassis.axes.left_stick_y.deadzone(0.1);

            mShootMode = Shoot.NONE;
            mEngageMode = Engage.NONE;
            mIsEngaged = false;
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
            else if (mMode == Mode.QRCODE_CENTRIC) { mLogger.info("==> MODE : QC"); }

            mLogger.info("======= COLLECTING =======");
            if(mIsEngaged) { mLogger.info("==> SHOOTING ENGAGED"); }
            else           { mLogger.info("==> SHOOTING NOT ENGAGED"); }
            //mLogger.metric("==> OUT VEL : ","" + mOuttakeWheels.getVelocity() / Math.PI * 180);
            //mLogger.metric("==> IN VEL : ","" + mIntakeBelts.getVelocity() / Math.PI * 180);
        }

    }

    public void loop() {

        mLocker.loop();
        move(mX,mY,mRotation);
        if ( mShootMode != Shoot.NONE )   { this.shoot(); }
        if ( mEngageMode != Engage.NONE ) { this.engage(); }

    }

    void engage(double velocity) {
        mEngageVelocity = velocity;
        this.engage();
    }

    void engage() {

        if (mEngageMode == Engage.NONE && !mIsEngaged ) { mEngageMode = Engage.WAITING; }
        else if (mEngageMode == Engage.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
            mOuttakeWheels.control(mEngageVelocity);
            if (mOuttakeWheels.isTransitioning() && (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK)) {
                mEngageMode = Engage.WHEELS;
            }
        }
        else if (mEngageMode == Engage.WHEELS && !mOuttakeLeverArm.isMoving() && !mOuttakeWheels.isTransitioning()) {
            mEngageMode = Engage.NONE;
            mIsEngaged = true;
        }
    }

    public void shoot() {

        if (mShootMode == Shoot.NONE && mIsEngaged ) { mShootMode = Shoot.WAITING; }
        else if (mShootMode == Shoot.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootMode = Shoot.ARM;
            }
        }
        else if (mShootMode == Shoot.ARM && !mOuttakeLeverArm.isMoving()) {
            mShootMode = Shoot.NONE;
            mIsEngaged = false;
        }
    }

    void control_chassis() {

        if (mGamepadChassis.buttons.a.pressed()) {
            mLogger.info("==> QRCODE MODE");
            mMode = Mode.QRCODE_CENTRIC;
        }
        if (mGamepadChassis.buttons.a.notPressed()) {
            mLogger.info("==> FALLBACK MODE");
            mMode = mFallbackMode;
        }
        if (mGamepadChassis.buttons.x.pressed()) {
            if(mFallbackMode == Mode.FIELD_CENTRIC)      {
                mLogger.info("==> ROBOT CENTRIC MODE");
                mFallbackMode = mMode = Mode.ROBOT_CENTRIC;
            }
            else if(mFallbackMode == Mode.ROBOT_CENTRIC)      {
                mLogger.info("==> FIELD CENTRIC MODE");
                mFallbackMode = mMode = Mode.FIELD_CENTRIC;
            }
        }

        if(mGamepadChassis.buttons.left_bumper.pressed()) { mIsPrecise = true; }
        else                                              { mIsPrecise = false; }

    }

    void control_attachments() {

        if (mGamepadAttachments.buttons.left_bumper.pressedOnce()) {
            mLogger.info("==> STR INTAKE");
            mIntakeBelts.start(1.0);
        }
        if (mGamepadAttachments.buttons.left_trigger.pressedOnce()){
            mLogger.info("==> STP INTAKE");
            mIntakeBelts.stop();
        }
        if (mGamepadAttachments.buttons.right_bumper.pressedOnce()) {
            mLogger.info("==> RVS INTAKE");
            mIntakeBelts.start(-1.0);
        }
        if (mGamepadAttachments.buttons.right_trigger.releasedOnce()){
            mLogger.info("==> STP INTAKE");
            mIntakeBelts.stop();
        }

        if (mGamepadAttachments.buttons.dpad_up.pressed()) {
            mLogger.info("==> SHT");
            Vector2d direction = mLocker.getDirection();
            if(direction != null) {
                double distance = direction.norm();
                double velocity = VelocityAbacus.getVelocity(distance);
                if (!mIsEngaged) { this.engage(velocity); }
                else             { this.shoot();          }
            }
        }
    }

    void move(double x, double y, double rotation) {

        double             multiplier = 0.9;
        if (mIsPrecise)  { multiplier = 0.3; }

        mLogger.info(String.format("\n==>  X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

        if(mMode == Mode.ROBOT_CENTRIC) {
            mChassis.drive(x,y,rotation, 0, multiplier);
        }
        else if (mMode == Mode.FIELD_CENTRIC)
        {
            double heading = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            heading += mHeadingOffset;
            mChassis.drive(x,y,rotation, heading, multiplier);
        }
        else if(mMode == Mode.QRCODE_CENTRIC) {
            if(mLocker.isSet()) {
                mChassis.drive(x, y, mLocker.getRotation(), mLocker.getHeading(), multiplier);
            }
        }
    }

    boolean initialize_collecting(Configuration config, HardwareMap hwm) {

        boolean result = true;

        mLogger.info("======= COLLECTING =======");

        mIntakeBelts        = new IntakeBelts();
        mOuttakeWheels      = new OuttakeWheels();
        mOuttakeLeverArm    = new OuttakeLeverArm();

        mIntakeBelts.setHW(config, hwm, mLogger);
        mOuttakeWheels.setHW(config, hwm, mLogger);
        mOuttakeLeverArm.setHW(config, hwm, mLogger);

        return result;

    }

    boolean initialize_vision(Configuration config, HardwareMap hwm) {

        boolean result = true;

        mLogger.info("========= VISION =========");
        String status = "";

        ConfLimelight limelight = config.getLimelight("limelight");
        if (limelight == null) { status += " LIME"; result = false; }
        else {
            mVision = new Vision(limelight, hwm, "vision", mLogger);
            mVision.initialize();
        }

        mCamera = new Camera();
        mCamera.setHW(Configuration.s_Current, hwm, mLogger);
        mCamera.setPosition(Camera.Position.TAG);

        if(result) { mLogger.info("==>  CONF : OK"); }
        else       { mLogger.warning("==>  CONF : KO : " + status); }

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
            if (imu != null) { mImu = hwm.tryGet(IMU.class, imu.getName()); }

            if (mImu == null)                { status += " IMU"; result = false; }
        }

        if (result) {

            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    imu.getLogo(), imu.getUsb());
            mImu.initialize(new IMU.Parameters(RevOrientation));
            mImu.resetYaw();

            mHeadingOffset = 0;
            Double initialHeading = config.retrieve("heading");
            if (initialHeading != null) {
                // From FTC field reference to initial robot position;
                mHeadingOffset = initialHeading;
            }
            mLogger.debug("==>  Heading Offset : " + mHeadingOffset);

            mDrive = new MecanumDrive(hwm, new Pose2d(0, 0, 0));

        }

        return result;

    }


}