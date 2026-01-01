/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   This class is responsible for maintaining the direction
   Of the target QR code along the robot movement.
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.pose;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Acmerobotics includes */
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfImu;
import org.firstinspires.ftc.teamcode.configurations.ConfLed;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.LedComponent;
import org.firstinspires.ftc.teamcode.components.LedCoupled;
import org.firstinspires.ftc.teamcode.components.LedMock;
import org.firstinspires.ftc.teamcode.components.LedSingle;

/* Roadrunner includes */
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Vision;

public class LockQRCode {

    static final int    s_WindowSize = 4;

    Logger              mLogger;
    boolean             mReady;
    boolean             mIsInFTC;

    PinpointLocalizer   mLocalizer;
    Vision              mVision;
    Path                mPath;

    Vector2d            mDirection;
    double              mRotation; // The rotation to give to the robot to reach QRCOde
    double              mHeading; // The direction along which the robot moves towards the QRCode

    LedComponent        mLed;

    double[]            mBuffer;
    int                 mIndex;
    int                 mCount;
    double              mSum;

    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Path path, Vision vision) {

        mLogger = logger;
        mLogger.info(Logger.Target.DRIVER_STATION, "======== LOCK QR CODE =========");

        mReady = true;
        mIsInFTC = false;
        String status = "";

        if (mReady) {
            mVision = vision;
            mPath = path;
            mDirection = null;
            mRotation = 0;
            mHeading = 0;
            mBuffer = new double[s_WindowSize];
            mIndex = 0;
            mCount = 0;
            mSum = 0;
            for (int i = 0; i < s_WindowSize; i ++) {
                mBuffer[i] = 0;
            }
        }

        if (mReady) {

            mLed = null;
            ConfLed led = config.getLed("tracking");
            if (led == null) { status += " LED"; }
            else {

                if (led.shallMock()) { mLed = new LedMock("tracking"); }
                else if (led.getHw().size() == 1) { mLed = new LedSingle(led, hwm, "tracking", mLogger); }
                else if (led.getHw().size() == 2) { mLed = new LedCoupled(led, hwm, "tracking", mLogger); }

                if (!mLed.isReady()) { status += " HW";}
            }

            mLocalizer = null;
            ConfImu pinpoint = config.getImu("pinpoint");
            if (pinpoint == null) { status += " PPT"; mReady = false; }
            else {
                PinpointLocalizer.PARAMS.parYTicks = pinpoint.getParY() / MecanumDrive.PARAMS.inPerTick;
                PinpointLocalizer.PARAMS.perpXTicks = pinpoint.getPerpX() / MecanumDrive.PARAMS.inPerTick;

                Pose2d pose = new Pose2d(0, 0, 0);
                mLocalizer = new PinpointLocalizer(hwm, pinpoint.getName(), MecanumDrive.PARAMS.inPerTick, pinpoint.getParReversed(), pinpoint.getPerpReversed(), pose);
            }
        }

        if (mReady) { mLogger.info("==>  LCK : OK"); }
        else { mLogger.warning("==>  LCK : KO : " + status); }
    }

    public boolean isSet() { return mIsInFTC; }

    public void loop() {

        if (mReady) {

            mLocalizer.update();

            Pose3D output = null;
            //output = mVision.getPosition();
            if (output != null) {

                Pose2d pose = new Pose2d(
                        -output.getPosition().x * Path.M_TO_INCHES,
                        -output.getPosition().y * Path.M_TO_INCHES,
                        (output.getOrientation().getYaw() + 180) * Math.PI / 180);

                mLocalizer.setPose(pose);
                mIsInFTC = true;
                if (mLed.isReady()) { mLed.on(LedComponent.Color.GREEN); }
            }

            if(mIsInFTC) {

                Pose2d qrcode = mPath.qrcode();
                Pose2d robot = mLocalizer.getPose();

                Vector2d pos_ftc = new Vector2d(qrcode.position.x - robot.position.x, qrcode.position.y - robot.position.y);
                double length = pos_ftc.norm();
                double theta1 = Math.atan2(pos_ftc.y,pos_ftc.x);
                double theta2 = qrcode.heading.toDouble() - theta1;

                mDirection = new Vector2d(length*Math.sin(theta2),length*Math.cos(theta2));
                double yaw = -Math.atan2(mDirection.x, mDirection.y);
                mRotation = robot.heading.toDouble() - yaw - qrcode.heading.toDouble() ;
                mHeading = robot.heading.toDouble() - theta1;

                mLogger.info(String.format("\n==> LCK RT: %2.2f HD: %2.2f",mRotation / Math.PI * 180, mHeading / Math.PI * 180));

                mSum -= mBuffer[mIndex];
                mBuffer[mIndex] = mRotation;
                mSum += mRotation;

                mIndex = (mIndex + 1) % s_WindowSize;
                if(mCount < s_WindowSize) {mCount ++;}

                mRotation = mSum / (mCount + 1e-10) / Math.PI * 3;

            }
        }
    }

    public Vector2d            getDirection() { return mDirection; }
    public double              getRotation()  { return mRotation;  }
    public double              getHeading()   { return mHeading;   }
}
