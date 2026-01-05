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
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.FtcDashboard;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.List;

@Config
public class LockQRCode {

    public static double COEFF_SPEED1 = 0.2;
    public static double COEFF_SPEED2 = 0.2;

    PIDFController.PIDFProvider    mP;
    PIDFController.PIDFProvider    mI;
    PIDFController.PIDFProvider    mD;
    PIDFController.PIDFProvider    mF;
    PIDFController.PIDFProvider    m1;
    PIDFController.PIDFProvider    m2;

    Logger              mLogger;
    boolean             mReady;
    boolean             mIsInFTC;

    PinpointLocalizer   mLocalizer;
    Vision              mVision;
    Path                mPath;

    Vector2d            mDirection;
    double              mRotation; // The rotation to give to the robot to reach QRCOde
    double              mRotation1; // The rotation to give to the robot to reach QRCOde
    double              mRotation2; // The rotation to give to the robot to reach QRCOde
    double              mHeading; // The direction along which the robot moves towards the QRCode

    LedComponent        mLed;

    List<PIDFController> mPIDs;

    double              mPCurrent;
    double              mICurrent;
    double              mDCurrent;
    double              mFCurrent;
    double              m1Current;
    double              m2Current;


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

            mP = new PIDFController.PIDFProvider(2);
            mI = new PIDFController.PIDFProvider(0);
            mD = new PIDFController.PIDFProvider(20);
            mF = new PIDFController.PIDFProvider(0);
            m1 = new PIDFController.PIDFProvider(-10);
            m2 = new PIDFController.PIDFProvider(10);

            mPIDs = new ArrayList<>();
            mPIDs.add(new PIDFController(mP.get(),mI.get(),mD.get(),mF.get(),m1.get(),m2.get()));
            mPIDs.add(new PIDFController(1,0,20,0,-10,10));

            mPCurrent = mP.get();
            mICurrent = mI.get();
            mDCurrent = mD.get();
            mFCurrent = mF.get();
            m1Current = m1.get();
            m2Current = m2.get();

            FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"P",mP);
            FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"I",mI);
            FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"D",mD);
            FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"F",mF);
            FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"1",m1);
            FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"2",m2);

            FtcDashboard.getInstance().updateConfig();
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

            if((Math.abs(mP.get() - mPCurrent) > 0.01) || (Math.abs(mI.get() - mICurrent) > 0.01) || (Math.abs(mD.get() - mDCurrent) > 0.01) || (Math.abs(mF.get() - mFCurrent) > 0.01) || (Math.abs(m1.get() - m1Current) > 0.01) || (Math.abs(m2.get() - m2Current) > 0.01)) {

                mPIDs.set(0,new PIDFController(mP.get(),mI.get(),mD.get(),mF.get(),m1.get(),m2.get()));

                mPCurrent = mP.get();
                mICurrent = mI.get();
                mDCurrent = mD.get();
                mFCurrent = mF.get();
                m1Current = m1.get();
                m2Current = m2.get();
            }

            mLocalizer.update();

            Pose3D output = null;
            output = mVision.getPosition();
            if (output != null) {

                Pose2d pose = new Pose2d(
                        -output.getPosition().x * Path.M_TO_INCHES,
                        -output.getPosition().y * Path.M_TO_INCHES,
                        (output.getOrientation().getYaw() + 180) * Math.PI / 180);
                mLogger.info(String.format("VIS : X: %2.2f Y: %2.2f HD : %2.2f" , pose.position.x,pose.position.y,pose.heading.toDouble() / Math.PI * 180));

                mLocalizer.setPose(pose);
                mIsInFTC = true;
                if ((mLed != null) && mLed.isReady()) { mLed.on(LedComponent.Color.GREEN); }
            }

            if(mIsInFTC) {

                Pose2d qrcode = mPath.qrcode();

                Pose2d robot = mLocalizer.getPose();
                mLogger.info(String.format("PPT : X: %2.2f Y: %2.2f HD : %2.2f" , robot.position.x,robot.position.y,robot.heading.toDouble() / Math.PI * 180));

                Vector2d pos_ftc = new Vector2d(qrcode.position.x - robot.position.x, qrcode.position.y - robot.position.y);
                double length = pos_ftc.norm();
                double theta1 = Math.atan2(pos_ftc.y,pos_ftc.x);
                double theta2 = qrcode.heading.toDouble() - theta1;

                mDirection = new Vector2d(length*Math.sin(theta2),length*Math.cos(theta2));
                double yaw = -Math.atan2(mDirection.x, mDirection.y);
                mLogger.trace("" + yaw);
                mRotation = robot.heading.toDouble() - yaw - qrcode.heading.toDouble() ;
                mHeading = robot.heading.toDouble() - theta1;

                mLogger.info(String.format("\n==> LCK RT: %2.2f HD: %2.2f",mRotation / Math.PI * 180, mHeading / Math.PI * 180));

                mRotation1 = mPIDs.get(0).update(mRotation,0,System.currentTimeMillis());
                mRotation2 = mPIDs.get(1).update(mRotation,0,System.currentTimeMillis());

                double rotation_predicted = (pos_ftc.y * mLocalizer.driver.getVelX(DistanceUnit.INCH) - pos_ftc.x * mLocalizer.driver.getVelY(DistanceUnit.INCH)) / length / length;

                mRotation1 -= COEFF_SPEED1 * rotation_predicted;
                mRotation2 -= COEFF_SPEED2 * rotation_predicted;

                mLogger.info(String.format("\n==> LCK RTPRED : %2.2f RT: %2.2f",rotation_predicted / Math.PI * 180, mRotation / Math.PI * 180));


            }

            mLogger.trace(Logger.Target.FILE,"after qrcode centric computation");

        }
    }

    public Vector2d            getDirection() { return mDirection; }
    public double              getRotation1()  { return mRotation1;  }
    public double              getRotation2()  { return mRotation2;  }
    public double              getHeading()   { return mHeading;   }
}
