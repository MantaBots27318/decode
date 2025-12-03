package org.firstinspires.ftc.teamcode;

/* Java includes */

/* Qualcomm includes */
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Intake includes */
import org.firstinspires.ftc.teamcode.intake.IntakeBrushes;

/* Outtake includes */
import org.firstinspires.ftc.teamcode.outtake.OuttakeWheels;
import org.firstinspires.ftc.teamcode.outtake.OuttakeLeverArm;

public class Collecting {

    public enum State {
        STOPPED,
        INTAKING,
        EJECTING,
        LOCKED,
        ENGAGED
    }

    public enum EngageMode {
        NONE,
        WAITING,
        WHEELS
    }

    public enum ShootingMode {
        NONE,
        WAITING,
        PRE,
        SHOOT,
        NEXT
    }
    public enum StartIntakeMode {
        NONE,
        WAITING,
        ARM
    }

    public enum EjectMode {
        NONE,
        WAITING,
        ARM,
        MOTORS
    }

    public enum StopIntakeMode {
        NONE,
        WAITING,
        ARM
    }

    public enum ShakeMode {
        NONE,
        WAITING,
        RAISE1,
        LOCK1,
        RAISE2,
        LOCK2
    }

    public enum CancelMode {
        NONE,
        WAITING,
        STOP_MOTORS,
        RAISE
    }


    Telemetry mLogger;

    StartIntakeMode mStartIntakeMode;
    EjectMode       mEjectMode;
    StopIntakeMode  mStopIntakeMode;
    ShootingMode    mShootingMode;
    ShakeMode       mShakeMode;
    CancelMode      mCancelMode;

    EngageMode      mEngageMode;

    IntakeBrushes   mIntakeBrushes;
    OuttakeWheels   mOuttakeWheels;
    OuttakeLeverArm mOuttakeLeverArm;

    Controller      mGamepad;

    double          mShootingPower;
    double          mEngagePower;

    State           mCurrentState;

    boolean         mIsCancellable;
    boolean         mIsCancelling;

    public Collecting() {

        mIntakeBrushes = new IntakeBrushes();

        mOuttakeWheels   = new OuttakeWheels();
        mOuttakeLeverArm = new OuttakeLeverArm();

        mStartIntakeMode    = StartIntakeMode.NONE;
        mStopIntakeMode     = StopIntakeMode.NONE;
        mShootingMode       = ShootingMode.NONE;
        mEjectMode          = EjectMode.NONE;
        mShakeMode          = ShakeMode.NONE;
        mEngageMode         = EngageMode.NONE;
        mCancelMode         = CancelMode.NONE;

        mIsCancellable      = false;
        mIsCancelling       = false;
    }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger, Controller gamepad) {

        mLogger = logger;
        mLogger.addLine("======= COLLECTING =======");

        mIntakeBrushes.setHW(config, hwm, mLogger);

        mOuttakeWheels.setHW(config, hwm, mLogger);
        mOuttakeLeverArm.setHW(config, hwm, mLogger);

        mGamepad = gamepad;
        mCurrentState = State.STOPPED;

        mIsCancellable = false;
        mIsCancelling = false;
    }

    public void control() {

        mLogger.addLine("======= COLLECTING =======");
        mLogger.addLine("-------- FUNCTION --------");

        if (mGamepad.buttons.left_bumper.pressedOnce()) {
            if (mCurrentState != State.INTAKING) {
                mLogger.addLine("==> STR INTAKE");
                start_intake();
            }
            else {
                mLogger.addLine("==> STP INTAKE");
                stop_intake();
            }
        }
        if (mGamepad.buttons.right_bumper.pressedOnce()) {
            if (mCurrentState != State.EJECTING) {
                mLogger.addLine("==> RVS INTAKE");
                eject();
            }
            else {
                mLogger.addLine("==> STP INTAKE");
                stop_intake();
            }
        }


        // SHOOTING FAR
        if (mGamepad.buttons.dpad_down.pressed()) {
            if (mCurrentState != State.ENGAGED) {
                engage(1.0);
            }
            else {
                shoot(0.92);
            }
        }

        // SHOOTING CLOSE
        if (mGamepad.buttons.dpad_up.pressed()) {
            if (mCurrentState != State.ENGAGED) {
                engage(0.85);
            }
            else {
                shoot(0.80);
            }
        }

        // CANCEL
        if (mGamepad.buttons.dpad_left.pressedOnce()) {
            cancel();
        }

        mLogger.addLine("\n--------- MOVING ---------");
        mLogger.addLine(this.logMovements());
    }

    public void loop() {
        if (mShootingMode != ShootingMode.NONE) {
            this.shoot();
        }
        if (mStartIntakeMode != StartIntakeMode.NONE) {
            this.start_intake();
        }
        if (mStopIntakeMode != StopIntakeMode.NONE) {
            this.stop_intake();
        }
        if (mEjectMode != EjectMode.NONE) {
            this.eject();
        }
        if(mShakeMode != ShakeMode.NONE) {
            this.shake();
        }
        if(mEngageMode != EngageMode.NONE) {
            this.engage();
        }
        if(mCancelMode != CancelMode.NONE) {
            this.cancel();
        }
    }

    public boolean shake() {

        mLogger.addLine("SHAKING : " + mShakeMode);

        if (mShakeMode == ShakeMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mShakeMode = ShakeMode.WAITING;
        }
        else if (mShakeMode == ShakeMode.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mShakeMode = ShakeMode.RAISE1;
            }
        }
        else if (mShakeMode == ShakeMode.RAISE1 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK) {
                mShakeMode = ShakeMode.LOCK1;
            }
        }
        else if (mShakeMode == ShakeMode.LOCK1 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT){
                mShakeMode = ShakeMode.RAISE2;
            }
        }
        else if (mShakeMode == ShakeMode.RAISE2 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK) {
                mShakeMode = ShakeMode.LOCK2;
            }
        }
        else if (mShakeMode == ShakeMode.LOCK2 && !mOuttakeLeverArm.isMoving()) {
            mShakeMode = ShakeMode.NONE;
        }

        return mShakeMode != ShakeMode.NONE;

    }

    public void cancel() {

        if (mIsCancellable && mCancelMode == CancelMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mCancelMode = CancelMode.WAITING;
            mIsCancelling = true;
        } else if (mCancelMode == CancelMode.WAITING) {
            mEngageMode = EngageMode.NONE;
            mShootingMode = ShootingMode.NONE;
            mOuttakeWheels.stop();
            mCancelMode = CancelMode.STOP_MOTORS;
        } else if ((mCancelMode == CancelMode.STOP_MOTORS) && !(mOuttakeWheels.isMoving()) && !(mOuttakeLeverArm.isMoving())) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT, 200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mCancelMode = CancelMode.RAISE;
            }
        } else if (mCancelMode == CancelMode.RAISE && !mOuttakeLeverArm.isMoving()) {
            mCancelMode = CancelMode.NONE;
            mCurrentState = State.STOPPED;
            mIsCancellable = false;
            mIsCancelling = false;
        }
    }

    public boolean engage(double power) {

        mEngagePower = power;
        mIsCancellable = true;
        return engage();
    }
    public boolean engage() {

       if (mEngageMode == EngageMode.NONE && !mIsCancelling && ((mCurrentState == State.STOPPED) || (mCurrentState == State.LOCKED))) {
            mEngageMode = EngageMode.WAITING;
            mIsCancellable = true;
        } else if (mEngageMode == EngageMode.WAITING) {
            mOuttakeWheels.start(mEngagePower, 3000);
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK, 200);
            if (mOuttakeWheels.isTransitioning() && (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK)) {
                mEngageMode = EngageMode.WHEELS;
            }
        } else if (mEngageMode == EngageMode.WHEELS && !mOuttakeLeverArm.isMoving() && !mOuttakeWheels.isTransitioning()) {
            mEngageMode = EngageMode.NONE;
            mCurrentState = State.ENGAGED;
        }

        return mEngageMode != EngageMode.NONE;
    }

    public void shoot(double power) {

       mShootingPower = power;
       shoot();

    }
    public void shoot() {

        mLogger.addLine("SHOOTING : " + mShootingMode);

        if (mCurrentState == State.ENGAGED && !mIsCancelling && mShootingMode == ShootingMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mIsCancellable = true;
            mShootingMode = ShootingMode.WAITING;
        }
        else if (mShootingMode == ShootingMode.WAITING && (mOuttakeLeverArm.getPosition() != OuttakeLeverArm.Position.NEXT)) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mShootingMode = ShootingMode.PRE;
            }
        }
        else if ((mShootingMode == ShootingMode.PRE) || (mShootingMode == ShootingMode.WAITING && (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT))) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootingMode = ShootingMode.SHOOT;
            }
        }
        else if (mShootingMode == ShootingMode.SHOOT && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT,200);
            mOuttakeWheels.start(mShootingPower, 2000);
            if ((mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) && (mOuttakeWheels.isTransitioning())){
                mShootingMode = ShootingMode.NEXT;
            }
        }
        else if(mShootingMode == ShootingMode.NEXT && !mOuttakeLeverArm.isMoving() && !(mOuttakeWheels.isTransitioning())) {
            mShootingMode = ShootingMode.NONE;
            mCurrentState = State.ENGAGED;
        }

    }

    public boolean start_intake() {

         mLogger.addLine("START INTAKE : " + mStartIntakeMode);

        if (((mCurrentState == State.STOPPED) || (mCurrentState == State.LOCKED) || (mCurrentState == State.EJECTING)) && mStartIntakeMode == StartIntakeMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mStartIntakeMode = StartIntakeMode.WAITING;
        }
        else if (mStartIntakeMode == StartIntakeMode.WAITING) {
            mOuttakeWheels.start(0.5, 200);
            mIntakeBrushes.start(1.0);
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.INTAKE,200);
            if ((mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.INTAKE) && (mOuttakeWheels.isTransitioning())) {
                mStartIntakeMode = StartIntakeMode.ARM;
            }
        }
        else if (mStartIntakeMode == StartIntakeMode.ARM && !mOuttakeLeverArm.isMoving() && !mOuttakeWheels.isTransitioning()) {
            mStartIntakeMode = StartIntakeMode.NONE;
            mCurrentState = State.INTAKING;
        }

        return mStartIntakeMode != StartIntakeMode.NONE;

    }

    public void eject() {

       mLogger.addLine("EJECT : " + mEjectMode);


        if (((mCurrentState == State.LOCKED) || (mCurrentState == State.STOPPED) || (mCurrentState == State.INTAKING)) && (mEjectMode == EjectMode.NONE)) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mEjectMode = EjectMode.WAITING;
        } else if (mEjectMode == EjectMode.WAITING) {
            mOuttakeWheels.start(-0.5, 200);
            mIntakeBrushes.start(-1.0);
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.INTAKE);
            if ((mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.INTAKE) && (mOuttakeWheels.isTransitioning())) {
                mEjectMode = EjectMode.ARM;
            }
        } else if (mEjectMode == EjectMode.ARM && !mOuttakeLeverArm.isMoving() && !mOuttakeWheels.isTransitioning()) {
            mEjectMode = EjectMode.NONE;
            mCurrentState = State.EJECTING;
        }

    }

    public boolean stop_intake() {

         mLogger.addLine("STOP INTAKE : " + mStopIntakeMode);

        if (((mCurrentState == State.INTAKING) || (mCurrentState == State.EJECTING)) && (mStopIntakeMode == StopIntakeMode.NONE)) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mStopIntakeMode = StopIntakeMode.WAITING;
        } else if (mStopIntakeMode == StopIntakeMode.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.LOCK);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.LOCK) {
                mStopIntakeMode = StopIntakeMode.ARM;
            }
        } else if (mStopIntakeMode == StopIntakeMode.ARM && !mOuttakeLeverArm.isMoving()) {
            mOuttakeWheels.stop();
            mIntakeBrushes.stop();
            mStopIntakeMode = StopIntakeMode.NONE;
            mCurrentState = State.LOCKED;
        }

        return mStopIntakeMode != StopIntakeMode.NONE;

    }


    public void startIntake() {
        mLogger.addLine("==> CFG : STARTING INTAKE");
        this.start_intake();
        while (mStartIntakeMode != StartIntakeMode.NONE){
            mLogger.addData("CFG : STARTING INTAKE", "IN");
            this.start_intake();
        }
    }


    public void shoot3(double power) {
        mLogger.addLine("==> CFG : SHOOTING 3");
        this.shoot(power);
        while (mShootingMode != ShootingMode.NONE){
            mLogger.addData("CFG : SHOOTING", "IN");
            this.shoot();
        }
        this.shoot(power);
        while (mShootingMode != ShootingMode.NONE){
            mLogger.addData("CFG : SHOOTING", "IN");
            this.shoot();
        }
        this.shoot(0);
        while (mShootingMode != ShootingMode.NONE){
            mLogger.addData("CFG : SHOOTING", "IN");
            this.shoot();
        }
        this.cancel();
        while(mCancelMode != CancelMode.NONE) {
            this.cancel();
        }
    }

    public void shoot4(double power) {
        mLogger.addLine("==> CFG : SHOOTING 3");
        this.shoot(power);
        while (mShootingMode != ShootingMode.NONE){
            mLogger.addData("CFG : SHOOTING", "IN");
            this.shoot();
        }
        this.shoot(power);
        while (mShootingMode != ShootingMode.NONE){
            mLogger.addData("CFG : SHOOTING", "IN");
            this.shoot();
        }
        this.shoot(power);
        while (mShootingMode != ShootingMode.NONE){
            mLogger.addData("CFG : SHOOTING", "IN");
            this.shoot();
        }
        mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT,1000);
        while (mOuttakeLeverArm.isMoving()) {
            try { Thread.sleep(10) ; }
            catch(Exception e) {}
        }

        this.cancel();
        while(mCancelMode != CancelMode.NONE) {
            this.cancel();
        }
    }

    public String logMovements() {
        String result = "";
        if (mIntakeBrushes.isMoving()) {
            result += "IN BRS\n";
        }
        if (mOuttakeWheels.isMoving()) {
            result += "OUT WHL\n";
        }
        if (mOuttakeLeverArm.isMoving()) {
            result += "OUT LVR\n";
        }
        return result;
    }

    public String logState() {
        String result = "";
        result += "ENG STATE : " + mEngageMode + "\n";
        result += "SHT STATE : " + mShootingMode + "\n";
        result += "CCL STATE : " + mCancelMode + "\n";
        result += "CLTG STATE : " + mCurrentState.toString() + "\n";
        result += "CLTG CANCELLABLE : " + mIsCancellable;
        result += "CLTG IS CANCELLING : " + mIsCancelling;

        return result;
    }

}