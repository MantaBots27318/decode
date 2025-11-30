package org.firstinspires.ftc.teamcode;

/* Java includes */

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;
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

    public enum ShootingMode {
        NONE,
        WAITING,
        STARTING_WHEELS,
        SHOOT1,
        NEXT1,
        SHOOT2,
        NEXT2,
        SHOOT3
    }
    public enum StartIntakeMode {
        NONE,
        WAITING,
        ARM,
        MOTORS
    }
    public enum StopIntakeMode {
        NONE,
        WAITING,
        ARM,
        MOTORS
    }


    Telemetry mLogger;

    StartIntakeMode mStartIntakeMode;
    StopIntakeMode  mStopIntakeMode;
    ShootingMode    mShootingMode;

    IntakeBrushes   mIntakeBrushes;
    OuttakeWheels   mOuttakeWheels;
    OuttakeLeverArm mOuttakeLeverArm;

    Controller mGamepad;



    public Collecting() {

        mIntakeBrushes = new IntakeBrushes();

        mOuttakeWheels   = new OuttakeWheels();
        mOuttakeLeverArm = new OuttakeLeverArm();

        mStartIntakeMode = StartIntakeMode.NONE;
        mStopIntakeMode  = StopIntakeMode.NONE;
        mShootingMode = ShootingMode.NONE;
    }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger, Controller gamepad) {

        mLogger = logger;
        mLogger.addLine("======= COLLECTING =======");

        mIntakeBrushes.setHW(config, hwm, mLogger);

        mOuttakeWheels.setHW(config, hwm, mLogger);
        mOuttakeLeverArm.setHW(config, hwm, mLogger);

        mGamepad = gamepad;
    }

    public void control() {

        mLogger.addLine("======= COLLECTING =======");
        mLogger.addLine("-------- FUNCTION --------");

        if (mGamepad.buttons.left_bumper.pressedOnce()) {
            if (!mIntakeBrushes.isMoving()) {
                mLogger.addLine("==> STR IN BRS");
                start_intake();
            } else {
                mLogger.addLine("==> STP IN BRS");
                stop_intake();
            }
        }

        if (mGamepad.buttons.dpad_up.pressedOnce()) { shoot(); }

        if (mGamepad.buttons.dpad_left.pressedOnce()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.OPEN);
            mOuttakeWheels.stop();
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
    }

    public void shoot() {

        mLogger.addLine("SHOOTING : " + mShootingMode);

        if (mShootingMode == ShootingMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mShootingMode = ShootingMode.WAITING;
        }
        else if (mShootingMode == ShootingMode.WAITING) {
            mOuttakeWheels.start(0.9);
            if (mOuttakeWheels.isTransitioning()) {
                mShootingMode = ShootingMode.STARTING_WHEELS;
            }
        }
        else if (mShootingMode == ShootingMode.STARTING_WHEELS && !mOuttakeWheels.isTransitioning()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootingMode = ShootingMode.SHOOT1;
            }
        }
        else if (mShootingMode == ShootingMode.SHOOT1 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT,2000);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mShootingMode = ShootingMode.NEXT1;
            }
        }
        else if (mShootingMode == ShootingMode.NEXT1 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootingMode = ShootingMode.SHOOT2;
            }
        }
        else if (mShootingMode == ShootingMode.SHOOT2 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT,2000);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mShootingMode = ShootingMode.NEXT2;
            }
        }
        else if (mShootingMode == ShootingMode.NEXT2 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT,200);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootingMode = ShootingMode.SHOOT3;
            }
        }
        else if (mShootingMode == ShootingMode.SHOOT3 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.OPEN);
            mOuttakeWheels.stop();
            mShootingMode = ShootingMode.NONE;
        }

    }



    public boolean start_intake() {
        mLogger.addLine("START INTAKE : " + mStartIntakeMode);

        if (mStartIntakeMode == StartIntakeMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mStartIntakeMode = StartIntakeMode.WAITING;
        }
        else if (mStartIntakeMode == StartIntakeMode.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.INTAKE);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.INTAKE) {
                mStartIntakeMode = StartIntakeMode.ARM;
            }
        }
        else if (mStartIntakeMode == StartIntakeMode.ARM && !mOuttakeLeverArm.isMoving()) {
            mOuttakeWheels.start(0.4);
            mIntakeBrushes.start(1.0);
            if (mOuttakeWheels.isTransitioning()) {
                mStartIntakeMode = StartIntakeMode.MOTORS;
            }
        }
        else if (mStartIntakeMode == StartIntakeMode.MOTORS && !mOuttakeWheels.isTransitioning()) {
            mStartIntakeMode = StartIntakeMode.NONE;
        }

        return mStartIntakeMode != StartIntakeMode.NONE;

    }

    public boolean stop_intake() {
        mLogger.addLine("STOP INTAKE : " + mStopIntakeMode);

        if (mStopIntakeMode == StopIntakeMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mStopIntakeMode = StopIntakeMode.WAITING;
        }
        else if (mStopIntakeMode == StopIntakeMode.WAITING) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mStopIntakeMode = StopIntakeMode.ARM;
            }
        }
        else if (mStopIntakeMode == StopIntakeMode.ARM && !mOuttakeLeverArm.isMoving()) {
            //mOuttakeWheels.stop();
            mIntakeBrushes.stop();
            mStopIntakeMode = StopIntakeMode.NONE;

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

    public void stopIntake() {
        mLogger.addLine("==> CFG : STOPPING INTAKE");
        this.stop_intake();
        while (mStartIntakeMode != StartIntakeMode.NONE){
            mLogger.addData("CFG : STOPPING INTAKE", "IN");
            this.stop_intake();
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

}