/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   robot driving commands
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode;

/* System includes */
import android.annotation.SuppressLint;

/* Qualcomm includes */
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Roadrunner includes */
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.Range;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.components.MotorComponent;

/* Roadrunner includes */
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Vision;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.utils.Logger;

/* Path includes */
import org.firstinspires.ftc.teamcode.pose.Path;

public class Driving {

    enum Mode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        QRCODE_CENTRIC
    }

    Logger                  mLogger;
    Vision                  mVision;
    boolean                 mReady;

    IMU                     mImu;
    GoBildaPinpointDriver   mPinpoint;

    double                  mHeadingOffset;
    Path                    mPath;

    MecanumDrive            mDrive;
    SmartTimer              mTimer;
    Action                  mAction;

    HardwareMap             mMap;

    MotorComponent          mFrontLeftMotor;
    MotorComponent          mBackLeftMotor;
    MotorComponent          mFrontRightMotor;
    MotorComponent          mBackRightMotor;

    Controller              mGamepad;

    boolean                 mIsAutomated = false;
    Mode                    mMode = Mode.FIELD_CENTRIC;
    Mode                    mFallbackMode;

    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Controller gp, Vision vision, Path path) {

        mLogger = logger;
        mLogger.info("======== DRIVING =========");

        mReady = true;

        mPath = path;

        // Get wheels and IMU parameters from configuration
        ConfMotor frontLeftWheel  = config.getMotor("front-left-wheel");
        ConfMotor frontRightWheel = config.getMotor("front-right-wheel");
        ConfMotor backLeftWheel   = config.getMotor("back-left-wheel");
        ConfMotor backRightWheel  = config.getMotor("back-right-wheel");
        ConfImu   imu             = config.getImu("built-in");
        ConfImu   pinpoint        = config.getImu("pinpoint");

        mTimer                    = new SmartTimer(mLogger);
        mVision                   = vision;
        mMap                      = hwm;

        if (mMode == Mode.FIELD_CENTRIC)        { mLogger.info("==>  FIELD CENTRIC"); }
        else if (mMode == Mode.ROBOT_CENTRIC)   { mLogger.info("==>  ROBOT CENTRIC"); }

        String status = "";
        if(frontLeftWheel == null)  { status += " FL";  mReady = false; }
        if(frontRightWheel == null) { status += " FR";  mReady = false; }
        if(backLeftWheel == null)   { status += " BL";  mReady = false; }
        if(backRightWheel == null)  { status += " BR";  mReady = false; }
        if(imu == null)             { status += " IMU"; mReady = false; }
        if(pinpoint == null)        { status += " PPT"; mReady = false; }

        if(mReady) { mLogger.info("==>  CONF : OK"); }
        else       { mLogger.warning("==>  CONF : KO : " + status); }

        if (mReady) {

            status = "";

            mFrontLeftMotor = new MotorSingle(frontLeftWheel, hwm, "front-left-wheel",mLogger);
            mBackLeftMotor = new MotorSingle(backLeftWheel, hwm, "back-left-wheel",mLogger);
            mFrontRightMotor = new MotorSingle(frontRightWheel, hwm, "front-right-wheel",mLogger);
            mBackRightMotor = new MotorSingle(backRightWheel, hwm, "back-right-wheel",mLogger);

            mImu = null;
            if(imu != null) { mImu = hwm.tryGet(IMU.class, imu.getName()); }
            mPinpoint = null;
            if(pinpoint != null) { mPinpoint = hwm.tryGet(GoBildaPinpointDriver.class, pinpoint.getName()); }

            if (!mFrontLeftMotor.isReady())     { status += " FL";  mReady = false; }
            if (!mFrontRightMotor.isReady())    { status += " FR";  mReady = false; }
            if (!mBackLeftMotor.isReady())      { status += " BL";  mReady = false; }
            if (!mBackRightMotor.isReady())     { status += " BR";  mReady = false; }
            if( mImu == null)                   { status += " IMU"; mReady = false; }
            if( mPinpoint == null)              { status += " PPT"; mReady = false; }

            mGamepad = gp;

            if(mReady) { logger.info("==>  HW : OK"); }
            else       { logger.warning("==>  HW : KO : " + status); }

        }

        if(mReady) {

            mFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    imu.getLogo(), imu.getUsb());
            mImu.initialize(new IMU.Parameters(RevOrientation));
            mImu.resetYaw();
            mHeadingOffset = 0;
            Double initialHeading = config.retrieve("heading");
            if(initialHeading != null) {
                // From FTC field reference to initial robot position;
                mHeadingOffset = initialHeading;
            }
            mLogger.debug("==>  Heading Offset : " + mHeadingOffset);

            mDrive                    = new MecanumDrive(hwm, new Pose2d(0,0,0));

        }



        if(mReady) {
            mGamepad.axes.left_stick_x.deadzone(0.1);
            mGamepad.axes.right_stick_x.deadzone(0.1);
            mGamepad.axes.left_stick_y.deadzone(0.1);
            mFallbackMode = mMode;
        }

        if(mReady) { logger.info("==>  READY"); }
        else       { logger.warning("==>  NOT READY"); }
    }


    @SuppressLint("DefaultLocale")
    public void control() {

        if(mReady) {

            if (mGamepad.buttons.b.pressedOnce() && mIsAutomated) {
                mLogger.info("==> AUT STOP");
                mIsAutomated = false;
            }
            if (mGamepad.buttons.a.pressed() && !mIsAutomated) {
                mLogger.info("==> QRCODE MODE");
                mMode = Mode.QRCODE_CENTRIC;
            }
            if (mGamepad.buttons.x.pressed() && !mIsAutomated) {
                mLogger.info("==> QRCODE MODE");
                if(mMode == Mode.FIELD_CENTRIC)      { mMode = Mode.ROBOT_CENTRIC; }
                else if(mMode == Mode.ROBOT_CENTRIC) { mMode = Mode.FIELD_CENTRIC; }
                mFallbackMode = mMode;
            }
            if (mGamepad.buttons.a.notPressed() && !mIsAutomated) {
                mLogger.info("==> FALLBACK MODE");
                mMode = mFallbackMode;
            }
            if (mGamepad.buttons.right_trigger.pressedOnce() && !mIsAutomated) {
                mLogger.info("==> AUT CLOSE SHT");
                shootPosition(Range.CLOSE);
            }
            if (mGamepad.buttons.right_bumper.pressedOnce() && !mIsAutomated) {
                mLogger.info("==> AUT FAR SHT");
                shootPosition(Range.FAR);
            }
            mDrive.localizer.update();

            mLogger.info("======== DRIVING =========");
            if(mMode == Mode.FIELD_CENTRIC) { mLogger.info("==> MODE : FC"); }
            else if(mMode == Mode.ROBOT_CENTRIC) { mLogger.info("==> MODE : RC"); }
            else if(mMode == Mode.QRCODE_CENTRIC) { mLogger.info("==> MODE : QC"); }

        }
        if (mReady && !mIsAutomated) {
            mLogger.info("==> CONTROLLED");

            double multiplier = 0.9;
            if (mGamepad.buttons.left_bumper.pressed())  { multiplier = 0.3; }

            double y = mGamepad.axes.left_stick_x.value();
            double x = mGamepad.axes.left_stick_y.value();
            double rotation = mGamepad.axes.right_stick_x.value();
            mLogger.info(String.format("\n==>  X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

            // Gather April Tags
            Pose3D output = mVision.getPosition();
            if (output != null) {

                Pose2d pose = new Pose2d(
                        -output.getPosition().x * Path.M_TO_INCHES,
                        -output.getPosition().y * Path.M_TO_INCHES,
                        (output.getOrientation().getYaw() + 180) * Math.PI / 180);

                mDrive.updatePose(pose);
            }

            if (mMode == Mode.FIELD_CENTRIC || mMode == Mode.QRCODE_CENTRIC) {
                double heading = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                heading += mHeadingOffset;

                if(mMode == Mode.QRCODE_CENTRIC) {
                    double length = Math.sqrt(((mPath.qrcode().x - mDrive.localizer.getPose().position.x) * (mPath.qrcode().x - mDrive.localizer.getPose().position.x)) + ((mPath.qrcode().y - mDrive.localizer.getPose().position.y) * (mPath.qrcode().y - mDrive.localizer.getPose().position.y)));
                    mLogger.info("" + length);
                    double theta1 = Math.atan2((mPath.qrcode().y - mDrive.localizer.getPose().position.y),(mPath.qrcode().x - mDrive.localizer.getPose().position.x));
                    mLogger.info("" + theta1);
                    double theta2 = 54-(theta1 * 180 / Math.PI);
                    mLogger.info("" + theta2);
                    double newX = length*Math.sin(theta2 / 180 * Math.PI);
                    mLogger.info("" + newX / Path.M_TO_INCHES);
                    double newY = length*Math.cos(theta2 / 180 * Math.PI);
                    mLogger.info("" + newY / Path.M_TO_INCHES);
                    output = mVision.getRelativePosition();
                    if(output != null) {
                        double yaw = -Math.atan2(-output.getPosition().x, -output.getPosition().z) / Math.PI * 180;
                        rotation = (heading + mPath.fieldCentric2FTC()) / Math.PI * 180 - yaw - 54;
                        heading = output.getOrientation().getYaw() * Math.PI / 180;
                        rotation = rotation / 180 * 10;
                        mLogger.info("" + mDrive.localizer.getPose().position);
                        mLogger.info("" + mDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);
                        mLogger.info("" + output.getPosition() + " " + output.getOrientation());
                        mLogger.info("" + mPath.qrcode());
                    }
                }
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-heading) + y * Math.sin(-heading);
                double rotY = - x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
                mLogger.info(String.format("==>  ROT: %2.2f HD : %6.1f X : %6.1f Y : %6.1f",rotation,heading /Math.PI * 180,x,y));
            }
            x *= 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
            double frontLeftPower = (x + y + rotation) / denominator * multiplier;
            double backLeftPower = (x - y + rotation) / denominator * multiplier;
            double frontRightPower = (x - y - rotation) / denominator * multiplier;
            double backRightPower = (x + y - rotation) / denominator * multiplier;

            mFrontLeftMotor.setPower(frontLeftPower);
            mBackLeftMotor.setPower(backLeftPower);
            mFrontRightMotor.setPower(frontRightPower);
            mBackRightMotor.setPower(backRightPower);
        }
        else if(mIsAutomated && mReady) {

            mLogger.info("==> AUTOMATED");
            mLogger.info(String.format("\n==>  X : %6.1f Y : %6.1f R:%6.1f", mDrive.getPose().position.x, mDrive.getPose().position.y, mDrive.getPose().heading.toDouble() / Math.PI / 180));
            mIsAutomated = mAction.run(new TelemetryPacket());
        }
    }

    public void shootPosition(Range range){

        // Gather April Tags
        Pose3D output = mVision.getPosition();
        mTimer.arm(100);

        while(output == null && mTimer.isArmed()) { output = mVision.getPosition(); }
        if (output != null) {

            Pose2d pose = new Pose2d(
                    -output.getPosition().x * Path.M_TO_INCHES,
                    -output.getPosition().y * Path.M_TO_INCHES,
                    (output.getOrientation().getYaw() + 180) * Math.PI / 180);

            mDrive.updatePose(pose);
            mDrive                    = new MecanumDrive(mMap, pose);

            if(range == Range.FAR) {

                double direction = Math.atan2(mPath.shootingFar().position.y - mDrive.getPose().position.y, mPath.shootingFar().position.x - mDrive.getPose().position.x);
                mAction = mDrive.actionBuilder(mDrive.getPose())
                        .setTangent(direction)
                        .splineToLinearHeading(mPath.shootingFar(), direction)
                        .build();
            }
            else if(range == Range.CLOSE) {

                double direction = Math.atan2(mPath.shootingClose().position.y - mDrive.getPose().position.y, mPath.shootingClose().position.x - mDrive.getPose().position.x);
                mAction = mDrive.actionBuilder(mDrive.getPose())
                        .setTangent(direction)
                        .splineToLinearHeading(mPath.shootingClose(), direction)
                        .build();
            }

            mIsAutomated = mAction.run(new TelemetryPacket());

        }


    }

}