/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   V2 robot commands
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Local includes */
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PositionMath;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

public class Robot {

    static final double    sDefaultMovementsMultiplier = 1.0;

    static final double    sPreciseMovementsMultiplier = 0.3;

    static final double    sGamepadChassisDeadZone     = 0.1;

    enum Mode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    Logger                  mLogger;
    boolean                 mReady;
    SmartTimer              mTimer;

    // Configuration
    double                  mHeadingOffset;
    Path                    mPath;
    Mode                    mMode = Mode.FIELD_CENTRIC;
    Mode                    mFallbackMode;
    boolean                 mPreciseMovements;
    Pose2d                  mTurretPositionInRR;

    // Subsystems
    Chassis                 mChassis;
    IntakeWheels            mIntake;
    Turret                  mTurret;
    Transfer                mTransfer;

    // Components
    IMU                     mImu;

    Controller              mGamepadChassis;
    Controller              mGamepadAttachments;

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
            mFallbackMode       = mMode;
            mPreciseMovements   = false;
        }
        if(mReady) {
            mTurretPositionInRR = config.getPosition("turret");
            if(mTurretPositionInRR == null) { mReady = false; }
        }
        if(mReady) { mReady = this.initialize_drive(config, hwm);      }
        if(mReady) { mReady = this.initialize_collecting(config, hwm, mChassis.getFTCPosition()); }


        if(mReady) {
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

            mLogger.info("======= COLLECTING =======");
        }

    }

    public void loop() {
        if(mReady) {
            move(mX, mY, mRotation);
            mTurret.loop(mChassis.getXVelocity(), mChassis.getYVelocity(), 0.04);
            Pose2d turret_ftc_position = mTurret.getFTCPosition();
            if (turret_ftc_position != null) {
                mChassis.setPosition(PositionMath.getRobotPoseFromLimelight(turret_ftc_position, mTurretPositionInRR));
            }
        }
    }

    void control_chassis() {

        if (mGamepadChassis.buttons.a.pressedOnce()) {
            if(mFallbackMode == Mode.FIELD_CENTRIC)      {
                mLogger.info("==> ROBOT CENTRIC MODE");
                mFallbackMode = mMode = Mode.ROBOT_CENTRIC;
            }
            else if(mFallbackMode == Mode.ROBOT_CENTRIC)      {
                mLogger.info("==> FIELD CENTRIC MODE");
                mFallbackMode = mMode = Mode.FIELD_CENTRIC;
            }
        }

        mPreciseMovements = mGamepadChassis.buttons.left_bumper.pressed();

    }

    void control_attachments() {

        if (mGamepadChassis.buttons.x.pressedOnce())   { start_stop_intake(); }
        if (mGamepadChassis.buttons.y.pressedOnce())   { reverse_stop_intake(); }
        if (mGamepadChassis.buttons.dpad_up.pressedOnce())  { mTurret.start_flyWheel(); }
        if (mGamepadChassis.buttons.dpad_up.pressedOnce())  { mTurret.stop_flyWheel(); }

        if(mGamepadChassis.buttons.b.pressedOnce()) {
            if(mTransfer.getPosition() == Transfer.Position.BLOCK) { mTransfer.setPosition(Transfer.Position.LET); }
            else if(mTransfer.getPosition() == Transfer.Position.LET) { mTransfer.setPosition(Transfer.Position.BLOCK); }
        }
        if(mGamepadChassis.buttons.right_bumper.pressedOnce()) {
            start_stop_shooting();
        }
    }

    void move(double x, double y, double rotation) {

        double                    multiplier = sDefaultMovementsMultiplier;
        if (mPreciseMovements)  { multiplier = sPreciseMovementsMultiplier; }

        mLogger.info(String.format("==>  X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

        if(mMode == Mode.ROBOT_CENTRIC) {
            mChassis.drive(x,y,rotation, 0, multiplier);
        }
        else if (mMode == Mode.FIELD_CENTRIC)
        {
            double heading = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - mHeadingOffset;
            mLogger.trace("yaw update : " + heading / Math.PI * 180);
            mChassis.drive(x,y,rotation, heading, multiplier);
        }
    }

    boolean initialize_collecting(Configuration config, HardwareMap hwm, Pose2d chassis) {

        boolean result = true;

        mLogger.info("======= COLLECTING =======");

        if(mReady) {

            mTurret = new Turret();
            Pose2d turret_position = PositionMath.getRobotPoseFromLimelight(
                    chassis,
                    new Pose2d(-mTurretPositionInRR.position.x, -mTurretPositionInRR.position.y, mTurretPositionInRR.heading.toDouble()));

            mTurret.setHW(config, hwm, mLogger, mPath, turret_position);

            mIntake = new IntakeWheels();
            mIntake.setHW(config, hwm, mLogger);

            mTransfer = new Transfer();
            mTransfer.setHW(config, hwm, mLogger);
        }

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

    public void start_stop_intake() {
        mLogger.info("==> STR INTAKE");
        if(mIntake.isMoving()) {
            if(mIntake.isReversed()) { mIntake.start(1.0); }
            else { mIntake.stop(); }
        }
        else {
            mIntake.start(1.0);
            mTurret.reset();
        }
    }

    public void reverse_stop_intake() {
        mLogger.info("==> RVS INTAKE");
        if(mIntake.isMoving()) {
            if(!mIntake.isReversed()) { mIntake.start(-1.0); }
            else { mIntake.stop(); }
        }
        else { mIntake.start(-1.0); }
    }


    public void start_stop_shooting() {
        mLogger.info("==> SHOOT");
        if(mTurret.isShooting()) {
            mTurret.stop();
        }
        else { mTurret.shoot(); }
    }

    public boolean start_engage() {
        return false;
    }

    public void shoot3() {}


}