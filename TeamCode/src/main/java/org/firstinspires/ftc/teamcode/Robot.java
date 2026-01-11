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
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/* Subsystem includes */
import org.firstinspires.ftc.teamcode.subsystems.IntakeBelts;
import org.firstinspires.ftc.teamcode.subsystems.IntakeEntryArm;
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
        ARM_AND_PUSH,
        ARM_AND_LET,
        WHEELS
    }

    public enum Shoot {
        NONE,
        WAITING,
        ARM,
        NEXT
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
    IntakeEntryArm          mIntakeEntryArm;
    double                  mTargetVelocity;
    double                  mTargetDistance;

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
    double                  mEngagedTargetVelocity;
    double                  mEngagedRealVelocity;
    double                  mEngagedDistance;
    boolean                 mIsEngagingFirst;

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
            mIsEngagingFirst = false;
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
            if(mIsEngaged) {
                mLogger.info("==> SHOOTING ENGAGED");
                mLogger.info("==> ENGAGED TARGET VELOCITY : " + mEngagedTargetVelocity);
                mLogger.info("==> ENGAGED CURRENT VELOCITY : " + mEngagedRealVelocity);
                mLogger.info("==> ENGAGED DISTANCE : " + mEngagedDistance);
                mLogger.info("==> ENGAGING FIRST : " + mIsEngagingFirst);
            }
            else           { mLogger.info("==> SHOOTING NOT ENGAGED"); }
            mLogger.metric("==> OUT VEL : ","" + mOuttakeWheels.getVelocity());
        }

    }

    public void loop() {

        mLocker.loop();
        move(mX,mY,mRotation);
        if ( mShootMode != Shoot.NONE )   { this.shoot(); }
        if ( mEngageMode != Engage.NONE ) { this.engage(); }

    }

    void engage(double velocity) {
        mTargetVelocity = velocity;
        this.engage();
    }

    boolean engage() {

        mLogger.trace("" + mEngageMode);

        if (mEngageMode == Engage.NONE && !mIsEngaged ) { mEngageMode = Engage.WAITING; }
        else if (mEngageMode == Engage.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK,200);
            if(!mIsEngagingFirst) { mIntakeEntryArm.setPosition(IntakeEntryArm.Position.PUSH,400); }
            mOuttakeWheels.control(mTargetVelocity, true);
            if ((mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK) && (mIsEngagingFirst || (mIntakeEntryArm.getPosition() == IntakeEntryArm.Position.PUSH)))  {
                mEngageMode = Engage.ARM_AND_PUSH;
            }
        }
        else if(mEngageMode == Engage.ARM_AND_PUSH && !mOuttakeLeverArm.isMoving() && !mIntakeEntryArm.isMoving()) {
            if(!mIsEngagingFirst) { mIntakeEntryArm.setPosition(IntakeEntryArm.Position.LET,200); }
            mOuttakeWheels.control(mTargetVelocity, false);
            if (mIsEngagingFirst || (mIntakeEntryArm.getPosition() == IntakeEntryArm.Position.LET))  {
                mEngageMode = Engage.ARM_AND_LET;
            }
        }
        else if(mEngageMode == Engage.ARM_AND_LET && !mIntakeEntryArm.isMoving()) {
            mOuttakeWheels.control(mTargetVelocity, false);
            if (!mOuttakeWheels.isTransitioning()) {
                mEngageMode = Engage.WHEELS;
            }
        }
        else if (mEngageMode == Engage.WHEELS) {
            mEngageMode = Engage.NONE;
            mIsEngaged = true;
            mIsEngagingFirst = false;
            mEngagedTargetVelocity = mTargetVelocity;
            mEngagedDistance = mTargetDistance;
            mEngagedRealVelocity = mOuttakeWheels.getVelocity();
        }

        return mEngageMode != Engage.NONE;
    }

    public boolean shoot() {

        mLogger.trace("" + mShootMode);

        if (mShootMode == Shoot.NONE && mIsEngaged ) { mShootMode = Shoot.WAITING; }
        else if (mShootMode == Shoot.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT, 200);
            mIntakeBelts.start(0.4);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootMode = Shoot.ARM;
            }
        }
        else if (mShootMode == Shoot.ARM && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT, 200);
            mIntakeBelts.start(-1);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootMode = Shoot.NEXT;
            }
        }
        else if (mShootMode == Shoot.NEXT && !mOuttakeLeverArm.isMoving()) {
            mShootMode = Shoot.NONE;
            mIsEngaged = false;
        }

        return mShootMode != Shoot.NONE;
    }

    void control_chassis() {

        if (mGamepadChassis.buttons.a.pressed()) {
            mLogger.info("==> QRCODE MODE A");
            mMode = Mode.QRCODE_CENTRIC;
            mShallCorrectSmallResidue = false;
        }
        else if (mGamepadChassis.buttons.b.pressed()) {
            mLogger.info("==> QRCODE MODE B");
            mMode = Mode.QRCODE_CENTRIC;
            mShallCorrectSmallResidue = true;
        }
        else if (mGamepadChassis.buttons.a.notPressed() && mGamepadChassis.buttons.b.notPressed()) {
            mLogger.info("==> FALLBACK MODE");
            mMode = mFallbackMode;
            mShallCorrectSmallResidue = false;
        }
        if (mGamepadChassis.buttons.x.pressed()) {
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

        if (mGamepadAttachments.buttons.left_bumper.pressedOnce())   { start_intake();   }
        if (mGamepadAttachments.buttons.left_bumper.releasedOnce())  { stop_intake();    }
        if (mGamepadAttachments.buttons.right_bumper.pressedOnce())  { reverse_intake(); }
        if (mGamepadAttachments.buttons.right_bumper.releasedOnce()) { stop_intake();    }

        if(mGamepadAttachments.buttons.dpad_up.pressedOnce()) {
            mIsEngagingFirst = true;
        }
        if(mGamepadAttachments.buttons.dpad_up.releasedOnce()) {
            mIntakeBelts.stop();
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
            mIntakeEntryArm.setPosition(IntakeEntryArm.Position.LET);
        }
        if (mGamepadAttachments.buttons.dpad_up.pressed()) {
            Vector2d direction = mLocker.getDirection();
            if(direction != null) {
                mTargetDistance = direction.norm();
                mLogger.trace("Distance : " + mTargetDistance);
                double velocity = VelocityAbacus.getVelocity(mTargetDistance);
                if (!mIsEngaged) { this.engage(velocity); }
                else             { this.shoot();          }
            }
        }
        if (mGamepadAttachments.buttons.dpad_left.pressedOnce()) {
            mOuttakeWheels.stop();
            mIntakeBelts.stop();
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
            mEngageMode = Engage.NONE;
            mShootMode = Shoot.NONE;
            mIsEngaged = false;
        }
        if (mGamepadAttachments.buttons.a.pressed()) {
            mIsEngaged = true;
            mEngageMode = Engage.NONE;
            this.shoot();
        }


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
            double heading = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            mLogger.trace("yaw : " + heading / Math.PI * 180);
            heading += mHeadingOffset;
            mChassis.drive(x,y,rotation, heading, multiplier);
        }
        else if(mMode == Mode.QRCODE_CENTRIC) {
            if(mLocker.isSet()) {

                if((Math.sqrt(x * x + y * y) < 0.01) && !mShallCorrectSmallResidue) {
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
        mOuttakeWheels      = new OuttakeWheels();
        mOuttakeLeverArm    = new OuttakeLeverArm();
        mIntakeEntryArm     = new IntakeEntryArm();

        mIntakeBelts.setHW(config, hwm, mLogger);
        mOuttakeWheels.setHW(config, hwm, mLogger);
        mOuttakeLeverArm.setHW(config, hwm, mLogger);
        mIntakeEntryArm.setHW(config,hwm,mLogger);

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


            mHeadingOffset = 0;
            Double initialHeading = config.retrieve("heading");
            if (initialHeading != null) {
                // From FTC field reference to initial robot position;
                mHeadingOffset = initialHeading;
            }
            mLogger.debug("==>  Heading Offset : " + mHeadingOffset);

            mDrive = new MecanumDrive(hwm, new Pose2d(0, 0, 0));

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
    public boolean stop_outtake() {
        mLogger.info("==> STP OUTTAKE");
        mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
        mOuttakeWheels.stop();
        return false;
    }
    public boolean start_engage(double velocity) {
        mIsEngagingFirst = true;
        this.engage(velocity);
        return mEngageMode != Engage.NONE;
    }

    public void shoot3(double velocity, Localizer localizer, int waitingTime) {
        mLogger.info(Logger.Target.DRIVER_STATION,"==> CFG : SHOOTING 4");
        if(waitingTime > 0) {
            mTimer.arm(waitingTime);
            while (mTimer.isArmed()) {
            }
        }
        this.shoot();
        while (mShootMode != Shoot.NONE){
            this.shoot();
           // localizer.update();
        }
        this.engage(velocity);
        while (mEngageMode != Engage.NONE){
            mLogger.info("CFG : ENGAGING");
            this.engage();
            //localizer.update();
        }
        this.shoot();
        while (mShootMode != Shoot.NONE){
            mLogger.info("CFG : SHOOTING");
            this.shoot();
            //localizer.update();
        }
        this.engage(velocity);
        while (mEngageMode != Engage.NONE){
            mLogger.info("CFG : ENGAGING");
            this.engage();
            localizer.update();
        }
        this.shoot();
        while (mShootMode != Shoot.NONE){
            mLogger.info("CFG : SHOOTING");
            this.shoot();
            localizer.update();
        }
        mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
        mOuttakeWheels.stop();
        mIntakeEntryArm.setPosition(IntakeEntryArm.Position.LET);

    }


}