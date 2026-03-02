/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   V2 robot commands
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Local includes */
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.Posable;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

public class Robot {

    static final double    sDefaultMovementsMultiplier = 1.0;
    static final double    sPreciseMovementsMultiplier = 0.3;
    static final double    sGamepadChassisDeadZone     = 0.1;
    static final double    sIntakePower                = 0.85;
    static final double    sGuidingPower               = 0.6;

    public enum Mode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        AUTONOMOUS
    }


    Logger                  mLogger;
    boolean                 mReady;
    SmartTimer              mTimer;

    // Configuration
    double                  mHeadingOffset;
    Path                    mPath;
    Mode                    mMode = Mode.FIELD_CENTRIC;
    boolean                 mPreciseMovements;

    // Relative position
    Pose2d                  mTurretPositionInRR;

    // Subsystems
    Chassis                 mChassis;
    Intake                  mIntake;
    Turret                  mTurret;
    Transfer                mTransfer;

    // Controllers
    Controller              mGamepadChassis;
    Controller              mGamepadAttachments;

    // Current displacement
    double                  mX;
    double                  mY;
    double                  mRotation;


    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Controller gamepad1, Controller gamepad2, Path path) {

        mLogger = logger;

        mReady = true;

        if(mReady) {
            mTimer              = new SmartTimer(mLogger);
            mGamepadChassis     = gamepad1;
            mGamepadAttachments = gamepad2;
            mPath               = path;
            mPreciseMovements   = false;

            LynxFirmware.throwIfModulesAreOutdated(hwm);

            for (LynxModule module : hwm.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        if(mReady) {

            if (mMode == Mode.FIELD_CENTRIC)      { mLogger.info("==>  FIELD CENTRIC"); }
            else if (mMode == Mode.ROBOT_CENTRIC) { mLogger.info("==>  ROBOT CENTRIC"); }

            mChassis = new Chassis();
            mChassis.setHW(config, hwm, mLogger);

            mHeadingOffset = mPath.FC2FTC();

        }

        if(mReady) {

            mTurret = new Turret();
            mTurret.setHW(config, hwm, mLogger, mPath);

            mIntake = new Intake();
            mIntake.setHW(config, hwm, mLogger);

            mTransfer = new Transfer();
            mTransfer.setHW(config, hwm, mLogger);

            mTurretPositionInRR = config.getPosition("turret");
            if(mTurretPositionInRR == null) { mReady = false; }

        }

        if(mReady && mGamepadChassis != null) {

            mGamepadChassis.axes.left_stick_x.deadzone(sGamepadChassisDeadZone);
            mGamepadChassis.axes.right_stick_x.deadzone(sGamepadChassisDeadZone);
            mGamepadChassis.axes.left_stick_y.deadzone(sGamepadChassisDeadZone);
        }

        if(mReady) { mLogger.info("==>  READY"); }
        else       { mLogger.warning("==>  NOT READY"); }
    }

    public void close()
    {
        mTurret.close();
    }

    public void initialize(Pose2d position, Mode mode) {

        if(mReady)
        {
            mMode = mode;
            if(position != null) {
                mChassis.setFTCPosition(position);
                mLogger.info("ROBOT FTC POSITION FROM INIT : " + position.position + " " + position.heading.toDouble() / Math.PI * 180);
                Pose2d turret_ftc_position = Posable.derivePose(position, new Pose2d(-mTurretPositionInRR.position.x, -mTurretPositionInRR.position.y, -mTurretPositionInRR.heading.toDouble()));
                mLogger.info("TURRET FTC POSITION FROM INIT : " + turret_ftc_position.position + " " + turret_ftc_position.heading.toDouble() / Math.PI * 180);
                mTurret.setFTCPosition(turret_ftc_position);
            }
        }

    }

    public void control() {

        if(mReady && mGamepadChassis != null) {

            if (mGamepadChassis.buttons.a.pressedOnce()) {
                if (mMode == Mode.FIELD_CENTRIC) {
                    mLogger.info("==> ROBOT CENTRIC MODE");
                    mMode = Mode.ROBOT_CENTRIC;
                } else if (mMode == Mode.ROBOT_CENTRIC) {
                    mLogger.info("==> FIELD CENTRIC MODE");
                    mMode = Mode.FIELD_CENTRIC;
                }
            }

            mPreciseMovements = mGamepadChassis.buttons.left_bumper.pressed();

            mY = mGamepadChassis.axes.left_stick_x.value();
            mX = mGamepadChassis.axes.left_stick_y.value();
            mRotation = mGamepadChassis.axes.right_stick_x.value();

        }
        if(mReady && mGamepadAttachments != null) {

            if (mGamepadAttachments.buttons.left_bumper.pressedOnce()) { start_stop_intake(); }
            if (mGamepadAttachments.buttons.left_trigger.pressedOnce()) { start_stop_flywheel(); }
            if (mGamepadAttachments.buttons.x.pressedOnce()) { reverse_stop_intake(); }
            if (mGamepadAttachments.buttons.right_bumper.pressedOnce()) {
                if (mTransfer.open()) { mTransfer.close(); }
                else { mTransfer.open_loop(); }
            }
        }

        if (mReady) {

            mLogger.info("======== DRIVING =========");
            if (mMode == Mode.FIELD_CENTRIC)       { mLogger.info("==> MODE : FC"); }
            else if (mMode == Mode.ROBOT_CENTRIC)  { mLogger.info("==> MODE : RC"); }

            mLogger.info("======= COLLECTING =======");
        }

    }

    public void loop() {

        if(mReady) {

            if(mTransfer.ongoing()) {mTransfer.open_loop(); }

            if(mMode != Mode.AUTONOMOUS) { move(mX, mY, mRotation); }

            Pose2d chassis_ftc_position = mChassis.getFTCPosition();
            if(chassis_ftc_position != null) {
                mLogger.metric("ROBOT FTC POSITION FROM PINPOINT : " ,""+ chassis_ftc_position.position + " " + chassis_ftc_position.heading.toDouble() / Math.PI * 180);
                Pose2d turret_ftc_position = Posable.derivePose(chassis_ftc_position, new Pose2d(-mTurretPositionInRR.position.x, -mTurretPositionInRR.position.y, -mTurretPositionInRR.heading.toDouble()));
                mLogger.metric("TURRET FTC POSITION FROM PINPOINT : " ,""+ turret_ftc_position.position + " " + turret_ftc_position.heading.toDouble() / Math.PI * 180);
                mTurret.setFTCPosition(turret_ftc_position);
            }

            mTurret.loop(mChassis.getXVelocity(), mChassis.getYVelocity(), 0.04);

            Pose2d turret_ftc_position = mTurret.getFTCPosition();
            if (turret_ftc_position != null) {
                mLogger.metric("TURRET FTC POSITION FROM LIMELIGHT : " , ""+ turret_ftc_position.position + " " + turret_ftc_position.heading.toDouble() / Math.PI * 180);
                Pose2d robot_ftc_position = Posable.derivePose(turret_ftc_position, mTurretPositionInRR);
                mLogger.metric("ROBOT FTC POSITION FROM LIMELIGHT : " ,""+ robot_ftc_position.position + " " + robot_ftc_position.heading.toDouble() / Math.PI * 180);
                mChassis.setFTCPosition(robot_ftc_position);
            }

        }

    }

    void move(double x, double y, double rotation) {

        if(mReady) {
            double multiplier = sDefaultMovementsMultiplier;
            if (mPreciseMovements) { multiplier = sPreciseMovementsMultiplier; }

            mLogger.info(String.format("==>  X : %6.1f Y : %6.1f R:%6.1f", x, y, rotation));

            if (mMode == Mode.ROBOT_CENTRIC) { mChassis.drive(x, y, rotation, 0, multiplier); }
            else if (mMode == Mode.FIELD_CENTRIC) {

                double heading = mChassis.getFTCPosition().heading.toDouble() - mHeadingOffset;
                mLogger.trace("yaw : " + heading / Math.PI * 180);
                mChassis.drive(x,y,rotation, heading, multiplier);
            }
        }
    }

    public void start_stop_intake() {
        mLogger.info("==> STR INTAKE");
        if(mIntake.isMoving()) {
            if(mIntake.isReversed()) { mIntake.start(sIntakePower,sGuidingPower); }
            else { mIntake.stop(); }
        }
        else { mIntake.start(sIntakePower,sGuidingPower); }
    }

    public void reverse_stop_intake() {
        mLogger.info("==> RVS INTAKE");
        if(mIntake.isMoving()) {
            if(!mIntake.isReversed()) { mIntake.start(-sIntakePower,-sGuidingPower); }
            else { mIntake.stop(); }
        }
        else { mIntake.start(-sIntakePower,sGuidingPower); }
    }


    public void start_stop_flywheel() {
        mLogger.info("==> SHOOT");
        if(mTurret.isShooting()) { mTurret.stop(); }
        else { mTurret.start(); }
    }

    public void shoot() {
        mTransfer.open_and_close_loop();
        while(mTransfer.ongoing()) { mTransfer.open_and_close_loop(); }
    }

}