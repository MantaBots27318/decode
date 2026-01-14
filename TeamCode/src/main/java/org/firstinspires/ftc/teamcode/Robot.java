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
import org.firstinspires.ftc.teamcode.components.LedComponent;
import org.firstinspires.ftc.teamcode.components.LedCoupled;
import org.firstinspires.ftc.teamcode.components.LedMock;
import org.firstinspires.ftc.teamcode.components.LedSingle;
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.ConfLed;
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

    enum Stop {
        NONE,
        WAITING,
        MOVING,
        OLAREADY,
        IEAREADY
    }

    Logger                  mLogger;
    boolean                 mReady;
    SmartTimer              mTimer;

    IntakeBelts             mIntakeBelts;
    OuttakeWheels           mOuttakeWheels;
    OuttakeLeverArm         mOuttakeLeverArm;
    IntakeEntryArm          mIntakeEntryArm;

    LedComponent            mLed1;
    LedComponent            mLed2;
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
    Stop                    mStopMode;
    boolean                 mIsEngaged;
    double                  mEngagedTargetVelocity;
    double                  mEngagedRealVelocity;
    double                  mEngagedDistance;
    boolean                 mIsEngagingReady;
    boolean                 mIsOuttaking;

    boolean                 mIsMoving;

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
        if(mReady) {
            String status = "";
            mLed1 = null;
            mLed2 = null;

            ConfLed led1 = config.getLed("tracking1");
            if (led1 == null) { status += " LED1"; }
            else {

                if (led1.shallMock()) { mLed1 = new LedMock("tracking1"); }
                else if (led1.getHw().size() == 1) { mLed1 = new LedSingle(led1, hwm, "tracking1", mLogger); }
                else if (led1.getHw().size() == 2) { mLed1 = new LedCoupled(led1, hwm, "tracking1", mLogger); }

                if (!mLed1.isReady()) { status += " HW";}

            }

            ConfLed led2 = config.getLed("tracking2");
            if (led2 == null) { status += " LED2"; }
            else {

                if (led2.shallMock()) { mLed2 = new LedMock("tracking2"); }
                else if (led2.getHw().size() == 1) { mLed2 = new LedSingle(led2, hwm, "tracking2", mLogger); }
                else if (led2.getHw().size() == 2) { mLed2 = new LedCoupled(led2, hwm, "tracking2", mLogger); }

                if (!mLed2.isReady()) { status += " HW";}
            }

            if (mReady) { mLogger.info("==>  LED : OK " + status); }
            else { mLogger.warning("==>  LED : KO : " + status); }
        }

        if(mReady) { mReady = this.initialize_collecting(config, hwm); }
        if(mReady) { mReady = this.initialize_vision(config, hwm);     }
        if(mReady) { mReady = this.initialize_drive(config, hwm);      }
        if(mReady) {
            mLocker = new LockQRCode();
            mLocker.setHW(config,hwm,mLogger,path,mVision, mLed1, mLed2);
        }


        if(mReady) {
            mGamepadChassis.axes.left_stick_x.deadzone(0.1);
            mGamepadChassis.axes.right_stick_x.deadzone(0.1);
            mGamepadChassis.axes.left_stick_y.deadzone(0.1);

            mShootMode = Shoot.NONE;
            mEngageMode = Engage.NONE;
            mStopMode = Stop.NONE;
            mIsEngaged = false;
            mIsOuttaking = false;
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
            if(mIsEngaged) {
                mLogger.info("==> SHOOTING ENGAGED");
                mLogger.info("==> ENGAGED TARGET VELOCITY : " + mEngagedTargetVelocity);
                mLogger.info("==> ENGAGED CURRENT VELOCITY : " + mEngagedRealVelocity);
                mLogger.info("==> ENGAGED DISTANCE : " + mEngagedDistance);
            }
            else           { mLogger.info("==> SHOOTING NOT ENGAGED"); }
            mLogger.metric("==> OUT VEL : ","" + mOuttakeWheels.getVelocity());
            mLogger.info("==> INTAKE OVERFLOW : " + mIntakeBelts.getDistance());
        }

    }

    public void loop() {

        mIsMoving = (Math.sqrt(mX * mX + mY * mY) >= 0.01);

        mLocker.loop();
        if(mIsOuttaking) {
            Vector2d direction = mLocker.getDirection();
            if (direction != null) {
                mTargetDistance = direction.norm();
                mLogger.trace("Distance : " + mTargetDistance);
                mTargetVelocity = VelocityAbacus.getVelocity(mTargetDistance);
                mOuttakeWheels.control(mTargetVelocity,mIsMoving);
            }
        }

        move(mX,mY,mRotation);

        mLed1.loop();
        mLed2.loop();

        if ( mShootMode != Shoot.NONE )   { this.shoot(); }
        if ( mEngageMode != Engage.NONE ) { this.engage_without_velocity(); }
        if ( mStopMode != Stop.NONE)      { this.stop();  }

    }

    void engage(double velocity) {
        mTargetVelocity = velocity;
        this.engage();
    }

    boolean engage() {


        if (mEngageMode == Engage.NONE && !mIsEngaged ) {
            mEngageMode = Engage.WAITING;
            mIsEngagingReady = false;
        }
        else if (mEngageMode == Engage.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK,200);
            mIntakeEntryArm.setPosition(IntakeEntryArm.Position.PUSH,400);
            mOuttakeWheels.control(mTargetVelocity, true);
            // Called just to follow the curve and launch engagement ending timer once we cross the threshold for the first time
            if(!mIsEngagingReady) { mIsEngagingReady = !(mOuttakeWheels.isTransitioning()); }
            if ((mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK) && (mIntakeEntryArm.getPosition() == IntakeEntryArm.Position.PUSH))  {
                mEngageMode = Engage.ARM_AND_PUSH;
                mLogger.trace("" + mEngageMode);
            }
        }
        else if(mEngageMode == Engage.ARM_AND_PUSH && !mOuttakeLeverArm.isMoving() && !mIntakeEntryArm.isMoving()) {
           mIntakeEntryArm.setPosition(IntakeEntryArm.Position.LET,200);
            mOuttakeWheels.control(mTargetVelocity, false);
            // Called just to follow the curve and launch engagement ending timer once we cross the threshold for the first time
            if(!mIsEngagingReady) { mIsEngagingReady = !(mOuttakeWheels.isTransitioning()); }
            if (mIntakeEntryArm.getPosition() == IntakeEntryArm.Position.LET)  {
                mEngageMode = Engage.ARM_AND_LET;
                mLogger.trace("" + mEngageMode);
            }
        }
        else if(mEngageMode == Engage.ARM_AND_LET && !mIntakeEntryArm.isMoving()) {
            mOuttakeWheels.control(mTargetVelocity, false);
            if(!mIsEngagingReady) { mIsEngagingReady = !(mOuttakeWheels.isTransitioning()); }
            if (mIsEngagingReady) {
                mEngageMode = Engage.WHEELS;
                mLogger.trace("" + mEngageMode);
            }
        }
        else if (mEngageMode == Engage.WHEELS) {
            mEngageMode = Engage.NONE;
            mLogger.trace("" + mEngageMode);
            mIsEngaged = true;
            mEngagedTargetVelocity = mTargetVelocity;
            mEngagedDistance = mTargetDistance;
            mEngagedRealVelocity = mOuttakeWheels.getVelocity();
        }

        return mEngageMode != Engage.NONE;
    }


    boolean engage_without_velocity() {

        if (mEngageMode == Engage.NONE && !mIsEngaged && mStopMode == Stop.NONE) {
            mEngageMode = Engage.WAITING;
            mIsEngagingReady = false;
        }
        else if (mEngageMode == Engage.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK,200);
            mIntakeEntryArm.setPosition(IntakeEntryArm.Position.PUSH,400);
            // Called just to follow the curve and launch engagement ending timer once we cross the threshold for the first time
            if(!mIsEngagingReady) { mIsEngagingReady = !(mOuttakeWheels.isTransitioning()); }
            if ((mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK) && (mIntakeEntryArm.getPosition() == IntakeEntryArm.Position.PUSH))  {
                mEngageMode = Engage.ARM_AND_PUSH;
                mLogger.trace("" + mEngageMode);
            }
        }
        else if(mEngageMode == Engage.ARM_AND_PUSH && !mOuttakeLeverArm.isMoving() && !mIntakeEntryArm.isMoving()) {
            mIntakeEntryArm.setPosition(IntakeEntryArm.Position.LET,200);
            // Called just to follow the curve and launch engagement ending timer once we cross the threshold for the first time
            if(!mIsEngagingReady) { mIsEngagingReady = !(mOuttakeWheels.isTransitioning()); }
            if (mIntakeEntryArm.getPosition() == IntakeEntryArm.Position.LET)  {
                mEngageMode = Engage.ARM_AND_LET;
                mLogger.trace("" + mEngageMode);
            }
        }
        else if(mEngageMode == Engage.ARM_AND_LET && !mIntakeEntryArm.isMoving()) {
            if(!mIsEngagingReady) { mIsEngagingReady = !(mOuttakeWheels.isTransitioning()); }
            if (mIsEngagingReady) {
                mEngageMode = Engage.WHEELS;
                mLogger.trace("" + mEngageMode);
            }
        }
        else if (mEngageMode == Engage.WHEELS) {
            mEngageMode = Engage.NONE;
            mLogger.trace("" + mEngageMode);
            mIsEngaged = true;
            mEngagedTargetVelocity = mTargetVelocity;
            mEngagedDistance = mTargetDistance;
            mEngagedRealVelocity = mOuttakeWheels.getVelocity();
        }

        return mEngageMode != Engage.NONE;
    }

    public boolean shoot() {



        if (mShootMode == Shoot.NONE && mIsEngaged ) {
            mShootMode = Shoot.WAITING;
            mLogger.trace("" + mShootMode);
        }
        else if (mShootMode == Shoot.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT, 200);
            mIntakeBelts.start(0.4);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootMode = Shoot.ARM;
                mLogger.trace("" + mShootMode);
            }
        }
        else if (mShootMode == Shoot.ARM && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT, 200);
            mIntakeBelts.start(-1);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootMode = Shoot.NEXT;
                mLogger.trace("" + mShootMode);
            }
        }
        else if (mShootMode == Shoot.NEXT && !mOuttakeLeverArm.isMoving()) {
            mShootMode = Shoot.NONE;
            mLogger.trace("" + mShootMode);
            mIsEngaged = false;
        }

        return mShootMode != Shoot.NONE;
    }
    void stop() {
        if (mStopMode == Stop.NONE) {
            mStopMode = Stop.WAITING;
            mIsEngaged = false;
            mEngageMode = Engage.NONE;
            mShootMode = Shoot.NONE;
            mLogger.trace("" + mStopMode);
            mOuttakeWheels.stopTransition();
        }
        if (mStopMode == Stop.WAITING) {
            mIntakeBelts.stop();
            mStopMode = Stop.MOVING;
            mLogger.trace("" + mStopMode);
        }

        if (mStopMode == Stop.MOVING && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK) {
                mStopMode = Stop.OLAREADY;
                mLogger.trace("" + mStopMode);
            }
        }
        if (mStopMode == Stop.OLAREADY && !mIntakeEntryArm.isMoving()) {
            mIntakeEntryArm.setPosition(IntakeEntryArm.Position.LET);
            if (mIntakeEntryArm.getPosition() == IntakeEntryArm.Position.LET) {
                mStopMode = Stop.IEAREADY;
                mLogger.trace("" + mStopMode);
            }
        }
        if (mStopMode == Stop.IEAREADY) {
            mStopMode = Stop.NONE;
            mLogger.trace("" + mStopMode);
        }


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

        if (mGamepadAttachments.buttons.left_trigger.pressedOnce())   { start_intake();   }
        if (mGamepadAttachments.buttons.left_trigger.releasedOnce())  { stop_intake();    }
        if (mGamepadAttachments.buttons.right_trigger.pressedOnce())  { reverse_intake(); }
        if (mGamepadAttachments.buttons.right_trigger.releasedOnce()) { stop_intake();    }

        if(mGamepadAttachments.buttons.dpad_up.releasedOnce() ) {
            stop();
        }
        if(mGamepadAttachments.buttons.dpad_up.pressedOnce() ) {
            // We stop updating the timeout if the robot is stopped, so that he can reach the end of its time
            // But if we start the outtake while stopped, it means it will never get a timeout set and might shoot right away
            // This line is to make sure that when we resume shooting, we set a timeout, which might have been deactivated by stop()
           mOuttakeWheels.control(VelocityAbacus.getVelocity(500),true);
        }
        if (mGamepadAttachments.buttons.dpad_up.pressed()) {
            mIsOuttaking = true;
            if (!mIsEngaged) { this.engage_without_velocity(); }
            else             { this.shoot();                   }
        }
        if (mGamepadAttachments.buttons.dpad_left.pressedOnce()) {
            mOuttakeWheels.stop();
            mIsOuttaking = false;
            stop();
        }
        if (mGamepadAttachments.buttons.a.pressedOnce()) {
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
        mOuttakeWheels      = new OuttakeWheels();
        mOuttakeLeverArm    = new OuttakeLeverArm();
        mIntakeEntryArm     = new IntakeEntryArm();

        mIntakeBelts.setHW(config, hwm, mLogger,mLed1, mLed2);
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

            mHeadingOffset = mPath.fieldCentric2FTC();

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
        this.engage(velocity);
        return mEngageMode != Engage.NONE;
    }

    public void shoot3(double velocity) {
        mLogger.info("CFG : SHOOTING");
        this.shoot();
        while (mShootMode != Shoot.NONE){
            this.shoot();
        }
        mLogger.info("CFG : ENGAGING");
        this.engage(velocity);
        while (mEngageMode != Engage.NONE){
            this.engage();
        }
        mLogger.info("CFG : SHOOTING");
        this.shoot();
        while (mShootMode != Shoot.NONE){
            this.shoot();
        }
        mLogger.info("CFG : ENGAGING");
        this.engage(velocity);
        while (mEngageMode != Engage.NONE){
            this.engage();
        }
        mLogger.info("CFG : SHOOTING");
        this.shoot();
        while (mShootMode != Shoot.NONE){
            this.shoot();
        }
        mLogger.info("CFG : STOPPING");
        mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
        mOuttakeWheels.stop();
        mIntakeEntryArm.setPosition(IntakeEntryArm.Position.LET);
        mIntakeBelts.stop();

    }


}