/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   V2 robot commands
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/* Local includes */
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.Posable;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

public class Robot {

    static final double    sDefaultMovementsMultiplier = 1.0;
    static final double    sPreciseMovementsMultiplier = 0.3;
    static final double    sGamepadChassisDeadZone     = 0.1;

    public enum Mode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        AUTONOMOUS
    }

    public enum TransferState {
        NONE,
        WAITING,
        DOWN,
        LET,
        BLOCK
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
    IntakeWheels            mIntake;
    Turret                  mTurret;
    Transfer                mTransfer;

    // Controllers
    Controller              mGamepadChassis;
    Controller              mGamepadAttachments;

    // Current displacement
    double                  mX;
    double                  mY;
    double                  mRotation;

    TransferState           mTransferState;


    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Controller gamepad1, Controller gamepad2, Path path) {

        mLogger = logger;

        mReady = true;

        if(mReady) {
            mTimer              = new SmartTimer(mLogger);
            mGamepadChassis     = gamepad1;
            mGamepadAttachments = gamepad2;
            mPath               = path;
            mPreciseMovements   = false;
            mTransferState      = TransferState.NONE;
        }
        if(mReady) {
            mTurretPositionInRR = config.getPosition("turret");
            if(mTurretPositionInRR == null) { mReady = false; }
        }
        if(mReady) { mReady = this.initialize_drive(config, hwm);      }
        if(mReady) { mReady = this.initialize_collecting(config, hwm, mChassis.getFTCPosition()); }


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
            if(mMode != Mode.AUTONOMOUS) { move(mX, mY, mRotation); }
            Pose2d chassis_ftc_position = mChassis.getFTCPosition();
            if(chassis_ftc_position != null) {
                mLogger.info("ROBOT FTC POSITION FROM PINPOINT : " + chassis_ftc_position.position + " " + chassis_ftc_position.heading.toDouble() / Math.PI * 180);
                Pose2d turret_ftc_position = Posable.derivePose(chassis_ftc_position, new Pose2d(-mTurretPositionInRR.position.x, -mTurretPositionInRR.position.y, -mTurretPositionInRR.heading.toDouble()));
                mLogger.info("TURRET FTC POSITION FROM PINPOINT : " + turret_ftc_position.position + " " + turret_ftc_position.heading.toDouble() / Math.PI * 180);
                mTurret.setFTCPosition(turret_ftc_position);
            }
            mTurret.loop(mChassis.getXVelocity(), mChassis.getYVelocity(), 0.04);
            Pose2d turret_ftc_position = mTurret.getFTCPosition();
            if (turret_ftc_position != null) {
                mLogger.info("TURRET FTC POSITION FROM LIMELIGHT : " + turret_ftc_position.position + " " + turret_ftc_position.heading.toDouble() / Math.PI * 180);
                Pose2d robot_ftc_position = Posable.derivePose(turret_ftc_position, mTurretPositionInRR);
                mLogger.info("ROBOT FTC POSITION FROM LIMELIGHT : " + robot_ftc_position.position + " " + robot_ftc_position.heading.toDouble() / Math.PI * 180);
                mChassis.setFTCPosition(robot_ftc_position);
            }
            if(mTransferState != TransferState.NONE) { transfer_cycle(); }
        }

    }

    void control_chassis() {

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
        }

    }

    void control_attachments() {

        if(mReady && mGamepadChassis != null) {
            if (mGamepadChassis.buttons.x.pressedOnce()) { start_stop_intake();   }
            if (mGamepadChassis.buttons.y.pressedOnce()) { reverse_stop_intake(); }
            if (mGamepadChassis.buttons.b.pressedOnce()) { transfer_cycle();      }
            if (mGamepadChassis.buttons.right_bumper.pressedOnce()) { start_stop_shooting(); }
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

    boolean initialize_collecting(Configuration config, HardwareMap hwm, Pose2d chassis) {

        mLogger.info("======= COLLECTING =======");

        if(mReady) {

            mTurret = new Turret();
            Pose2d turret_position = Posable.derivePose(
                    chassis,
                    new Pose2d(-mTurretPositionInRR.position.x, -mTurretPositionInRR.position.y, -mTurretPositionInRR.heading.toDouble()));

            mTurret.setHW(config, hwm, mLogger, mPath, turret_position);

            mIntake = new IntakeWheels();
            mIntake.setHW(config, hwm, mLogger);

            mTransfer = new Transfer();
            mTransfer.setHW(config, hwm, mLogger);
        }

        return true;

    }

    boolean initialize_drive(Configuration config, HardwareMap hwm) {

        mLogger.info("======== DRIVING =========");

        if (mMode == Mode.FIELD_CENTRIC)      { mLogger.info("==>  FIELD CENTRIC"); }
        else if (mMode == Mode.ROBOT_CENTRIC) { mLogger.info("==>  ROBOT CENTRIC"); }

        if(mReady) {
            mChassis = new Chassis();
            mChassis.setHW(config, hwm, mLogger);
        }

        return true;

    }

    public void start_stop_intake() {
        mLogger.info("==> STR INTAKE");
        if(mIntake.isMoving()) {
            if(mIntake.isReversed()) { mIntake.start(1.0); }
            else { mIntake.stop(); }
        }
        else {
            mIntake.start(1.0);
            //mTurret.reset();
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
        else { mTurret.start(); }
    }

    public void shoot() {
        transfer_cycle();
        while(mTransferState != TransferState.NONE) { transfer_cycle(); }
    }


    public void transfer_cycle() {

        if (mTransferState == TransferState.NONE) {
            mTransferState = TransferState.WAITING;
        }
        else if (mTransferState == TransferState.WAITING) {
            mTransfer.setPosition(Transfer.Position.DOWN);
            if (mTransfer.getPosition() == Transfer.Position.DOWN)  {
                mTransferState = TransferState.DOWN;
            }
        }
        else if(mTransferState == TransferState.DOWN && !mTransfer.isMoving()) {
            mTransfer.setPosition(Transfer.Position.LET,3000);
            if (mTransfer.getPosition() == Transfer.Position.LET)  {
                mTransferState = TransferState.LET;
            }
        }
        else if(mTransferState == TransferState.LET && !mTransfer.isMoving()) {
            mTransfer.setPosition(Transfer.Position.BLOCK);
            if (mTransfer.getPosition() == Transfer.Position.BLOCK)  {
                mTransferState = TransferState.BLOCK;
            }
        }
        else if (mTransferState == TransferState.BLOCK && !mTransfer.isMoving()) {
            mTransferState = TransferState.NONE;
        }

    }

}