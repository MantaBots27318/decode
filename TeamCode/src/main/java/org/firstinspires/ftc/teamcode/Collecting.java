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
    public enum IntakeMode {
        NONE,
        WAITING,
        OPEN,
        INTAKE
    }
    public enum NextMode {
        NONE,
        WAITING,
        BACK,
        FORTH
    }


    Telemetry mLogger;

    IntakeMode      mIntakeMode;
    ShootingMode    mShootingMode;
    NextMode        mNextMode;

    IntakeBrushes   mIntakeBrushes;
    OuttakeWheels   mOuttakeWheels;
    OuttakeLeverArm mOuttakeLeverArm;

    Controller mGamepad;



    public Collecting() {

        mIntakeBrushes = new IntakeBrushes();

        mOuttakeWheels   = new OuttakeWheels();
        mOuttakeLeverArm = new OuttakeLeverArm();

        mIntakeMode = IntakeMode.NONE;
        mShootingMode = ShootingMode.NONE;
        mNextMode = NextMode.NONE;
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
                mIntakeBrushes.start(0.9);
            } else {
                mLogger.addLine("==> STP IN BRS");
                mIntakeBrushes.stop();
                mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT);
            }
        }

        if (mGamepad.buttons.dpad_up.pressedOnce()) { shoot(); }

        if (mGamepad.buttons.dpad_down.pressedOnce()) { next();}

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
        if (mNextMode != NextMode.NONE) {
            this.next();
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
            mOuttakeWheels.start(0.8);
            if (mOuttakeWheels.isTransitioning()) {
                mShootingMode = ShootingMode.STARTING_WHEELS;
            }
        }
        else if (mShootingMode == ShootingMode.STARTING_WHEELS && !mOuttakeWheels.isTransitioning()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootingMode = ShootingMode.SHOOT1;
            }
        }
        else if (mShootingMode == ShootingMode.SHOOT1 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mShootingMode = ShootingMode.NEXT1;
            }
        }
        else if (mShootingMode == ShootingMode.NEXT1 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.SHOOT) {
                mShootingMode = ShootingMode.SHOOT2;
            }
        }
        else if (mShootingMode == ShootingMode.SHOOT2 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT);
            if (mOuttakeLeverArm.getPosition() == OuttakeLeverArm.Position.NEXT) {
                mShootingMode = ShootingMode.NEXT2;
            }
        }
        else if (mShootingMode == ShootingMode.NEXT2 && !mOuttakeLeverArm.isMoving()) {
            mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.SHOOT);
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

    public void next() {
        mLogger.addLine("NEXT : " + mNextMode);

        if (mNextMode == NextMode.NONE) {
            // Just transition to make sure that even though the first robot part is not yet ready
            // to move, we won't forget we have to keep on transiting
            mNextMode = NextMode.WAITING;
        }
        else if (mNextMode == NextMode.WAITING) {

            mNextMode = NextMode.NONE;
        }

    }

    public void startIntake() {
        mLogger.addLine("==> STR IN BRS");
        mIntakeBrushes.start(0.9);
    }

    public void stopIntake() {
        mLogger.addLine("==> STR IN BRS");
        mOuttakeLeverArm.setPosition(OuttakeLeverArm.Position.NEXT);
        mIntakeBrushes.stop();

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