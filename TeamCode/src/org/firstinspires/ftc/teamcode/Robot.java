/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

public class Robot {

    static final double sDefaultMultiplier       = 1.0;
    static final double sPreciseMultiplier1      = 0.5;
    static final double sPreciseMultiplier2      = 0.3;
    static final double sDeadZone                = 0.1;

    public enum Mode { ROBOT_CENTRIC, FIELD_CENTRIC }

    Logger      mLogger;
    boolean     mReady;
    SmartTimer  mTimer;
    Path        mPath;
    Mode        mMode = Mode.FIELD_CENTRIC;
    double      mHeadingOffset;
    boolean     mPrecise1;
    boolean     mPrecise2;
    double      mX, mY, mRotation;

    Chassis         mChassis;
    Controller      mGamepad1;
    Controller      mGamepad2;

    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Controller gamepad1, Controller gamepad2, Path path) {

        mLogger  = logger;
        mReady   = true;
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mPath    = path;
        mTimer   = new SmartTimer(mLogger);

        for (LynxModule module : hwm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        mChassis = new Chassis();
        mChassis.setHW(config, hwm, mLogger);

        mHeadingOffset = mPath.FC2FTC();

        if (mGamepad1 != null) {
            mGamepad1.axes.left_stick_x.deadzone(sDeadZone);
            mGamepad1.axes.right_stick_x.deadzone(sDeadZone);
            mGamepad1.axes.left_stick_y.deadzone(sDeadZone);
        }

        mLogger.info(mReady ? "==>  READY" : "==>  NOT READY");
    }

    public void close() {}

    public void initialize(Pose2d position, Mode mode) {
        if (mReady) {
            mMode = mode;
            if (position != null) {
                mChassis.setFTCPosition(position);
            }
        }
    }

    public void control() {
        if (mReady && mGamepad1 != null) {

            if (mGamepad1.buttons.y.pressedOnce()) {
                mMode = (mMode == Mode.FIELD_CENTRIC) ? Mode.ROBOT_CENTRIC : Mode.FIELD_CENTRIC;
                mLogger.info(mMode == Mode.FIELD_CENTRIC ? "==> FIELD CENTRIC" : "==> ROBOT CENTRIC");
            }

            mPrecise1  = mGamepad1.buttons.right_bumper.pressed();
            mPrecise2  = mGamepad1.buttons.right_trigger.pressed();
            mY         = mGamepad1.axes.left_stick_x.value();
            mX         = mGamepad1.axes.left_stick_y.value();
            mRotation  = mGamepad1.axes.right_stick_x.value();
        }
    }

    public void loop() {
        if (mReady) {
            move(mX, mY, mRotation);

            Pose2d pos = mChassis.getFTCPosition();
            if (pos != null) {
                mLogger.metric("ROBOT POSITION", pos.position + " " + pos.heading.toDouble() / Math.PI * 180);
            }
        }
    }

    void move(double x, double y, double rotation) {
        if (mReady) {
            double multiplier = sDefaultMultiplier;
            if (mPrecise1) multiplier = sPreciseMultiplier1;
            if (mPrecise2) multiplier = sPreciseMultiplier2;

            if (mMode == Mode.ROBOT_CENTRIC) {
                mChassis.drive(x, y, rotation, 0, multiplier);
            } else {
                double heading = mChassis.getFTCPosition().heading.toDouble() - mHeadingOffset;
                mChassis.drive(x, y, rotation, heading, multiplier);
            }
        }
    }
}