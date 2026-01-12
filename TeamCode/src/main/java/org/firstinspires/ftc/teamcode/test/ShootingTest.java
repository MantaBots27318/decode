/* -------------------------------------------------------
   Copyright (c) [2026] FASNY
   All rights reserved
   -------------------------------------------------------
   Speed test : opmode to test motor speed PID
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.test;

/* Java includes */
import java.io.IOException;
import java.io.FileWriter;

/* Qualcomm includes */
import android.os.Environment;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.LedComponent;
import org.firstinspires.ftc.teamcode.components.LedCoupled;
import org.firstinspires.ftc.teamcode.components.LedMock;
import org.firstinspires.ftc.teamcode.components.LedSingle;
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.ServoCoupled;
import org.firstinspires.ftc.teamcode.components.ServoMock;
import org.firstinspires.ftc.teamcode.components.ServoSingle;

/*  Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.ConfLed;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

/* Utils includes */
import org.firstinspires.ftc.teamcode.pose.LockQRCode;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
@TeleOp
public class ShootingTest extends OpMode {

    public static String MOTOR_OUTTAKE="outtake-wheels";
    public static double VELOCITY_OUTTAKE;
    public static String MOTOR_INTAKE="intake-belts";
    public static double VELOCITY_INTAKE;
    public static String SERVO_OUTTAKE="outtake-lever-arm";
    public static double POSITION_OUTTAKE;

    MotorComponent  mMotorOuttake = null;
    LedComponent mLed;
    MotorComponent  mMotorIntake = null;
    ServoComponent  mServoOuttake = null;
    Logger          mLogger;

    double          mSpeedOuttake = 0;
    double          mSpeedIntake = 0;
    double          mCurrentSpeedOuttake = 0;
    double          mCurrentSpeedIntake = 0;
    double          mPositionOuttake = 0;

    LockQRCode      mLocker;

    Path            mPath;

    Vision          mVision;

    PIDFController.PIDFProvider    mP;
    PIDFController.PIDFProvider    mI;
    PIDFController.PIDFProvider    mD;
    PIDFController.PIDFProvider    mF;
    PIDFCoefficients               mCoef;

    double          mPCurrent;
    double          mICurrent;
    double          mDCurrent;
    double          mFCurrent;

    FileWriter      mFile;

    @Override
    public void init() {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"speed-test");

        ConfMotor confmo = Configuration.s_Current.getMotor(MOTOR_OUTTAKE);
        if(confmo != null) {
            if (confmo.shallMock()) { mMotorOuttake = new MotorMock(MOTOR_OUTTAKE); }
            else if (confmo.getHw().size() == 1) { mMotorOuttake = new MotorSingle(confmo, hardwareMap, MOTOR_OUTTAKE, mLogger); }
            else if (confmo.getHw().size() == 2) { mMotorOuttake = new MotorCoupled(confmo, hardwareMap, MOTOR_OUTTAKE, mLogger); }

            if(mMotorOuttake != null) {
                mCoef = new PIDFCoefficients(300,3,0,0);

                mPCurrent = mCoef.p;
                mICurrent = mCoef.i;
                mDCurrent = mCoef.d;
                mFCurrent = mCoef.f;

                mP = new PIDFController.PIDFProvider(mCoef.p);
                mI = new PIDFController.PIDFProvider(mCoef.i);
                mD = new PIDFController.PIDFProvider(mCoef.d);
                mF = new PIDFController.PIDFProvider(mCoef.f);

                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"P",mP);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"I",mI);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"D",mD);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"F",mF);

                FtcDashboard.getInstance().updateConfig();
            }


        }


        if(confmo == null) { mLogger.warning("Could not find motor named " + MOTOR_OUTTAKE + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mMotorOuttake== null) { mLogger.warning("Motor outtake not initialized"); }

        mLed = null;
        ConfLed led = Configuration.s_Current.getLed("tracking");
        if (led != null) {

            if (led.shallMock()) { mLed = new LedMock("tracking"); }
            else if (led.getHw().size() == 1) { mLed = new LedSingle(led, hardwareMap, "tracking", mLogger); }
            else if (led.getHw().size() == 2) { mLed = new LedCoupled(led, hardwareMap, "tracking", mLogger); }

        }
        if(led == null) { mLogger.warning("Could not find led named intake in configuration " + Configuration.s_Current.getVersion()); }
        if(mLed == null) { mLogger.warning("Led not initialized"); }



        ConfLimelight confli = Configuration.s_Current.getLimelight("limelight");
        if(confli != null) {

            mVision = new Vision(confli,hardwareMap,"vision",mLogger);
            if(mVision != null) { mVision.initialize(); }
            mPath = new PathAutonomousGoal(mLogger);
            mPath.initialize(Alliance.RED,true);
            mLocker = new LockQRCode();
            mLocker.setHW(Configuration.s_Current,hardwareMap,mLogger,mPath,mVision,mLed);

        }


        if(confli == null) { mLogger.warning("Could not find limelight named limelight in configuration " + Configuration.s_Current.getVersion()); }
        if(mVision == null) { mLogger.warning("Vision not initialized"); }
        if(mLocker == null) { mLogger.warning("Locker not initialized"); }


        ConfMotor confmi = Configuration.s_Current.getMotor(MOTOR_INTAKE);
        if(confmi != null) {
            if (confmi.shallMock()) { mMotorIntake = new MotorMock(MOTOR_INTAKE); }
            else if (confmi.getHw().size() == 1) { mMotorIntake = new MotorSingle(confmi, hardwareMap, MOTOR_INTAKE, mLogger); }
            else if (confmi.getHw().size() == 2) { mMotorIntake = new MotorCoupled(confmi, hardwareMap, MOTOR_INTAKE, mLogger); }

        }

        if(confmi == null) { mLogger.warning("Could not find motor named " + MOTOR_INTAKE + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mMotorIntake== null) { mLogger.warning("Motor intake not initialized"); }


        String filepath = Environment.getExternalStorageDirectory().getPath()
                + "/FIRST/shooting-test.csv";
        mFile = null;
        try {
            mFile = new FileWriter(filepath);
            mFile.append("time,command,speed,p,i,d,f,position\n");
        } catch (IOException e) {
            mLogger.warning(e.getMessage());
        }

        ConfServo confs = Configuration.s_Current.getServo(SERVO_OUTTAKE);
        if(confs != null) {
            if (confs.shallMock()) { mServoOuttake = new ServoMock(SERVO_OUTTAKE); }
            else if (confs.getHw().size() == 1) { mServoOuttake = new ServoSingle(confs, hardwareMap, SERVO_OUTTAKE, mLogger); }
            else if (confs.getHw().size() == 2) { mServoOuttake = new ServoCoupled(confs, hardwareMap, SERVO_OUTTAKE, mLogger); }

            if(mServoOuttake != null) { mServoOuttake.setPosition(POSITION_OUTTAKE); }
        }

        if(confs == null) { mLogger.warning("Could not find servo named " + SERVO_OUTTAKE + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mServoOuttake== null) { mLogger.warning("Servo not initialized"); }

        mLogger.update();

    }

    @Override
    public void loop() {

        if(mMotorOuttake != null) {

            if(Math.abs(mSpeedOuttake - VELOCITY_OUTTAKE) > 0.01) {
                mMotorOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mMotorOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,mCoef);
                mMotorOuttake.setVelocity(VELOCITY_OUTTAKE);
                mSpeedOuttake = VELOCITY_OUTTAKE;


                mLogger.info("Changing Speed");
            }

            mCurrentSpeedOuttake = mMotorOuttake.getVelocity();
            mLogger.metric("COMMAND_OUTTAKE",""+mSpeedOuttake);
            mLogger.metric("VELOCITY_OUTTAKE", ""+mCurrentSpeedOuttake);

        }

        if (mLocker != null) {

            mLogger.trace("here");
            mLocker.loop();
            Vector2d direction = mLocker.getDirection();
            if(direction != null) {
                mLogger.metric("DISTANCE", ""+ direction.norm());
            }

        }

        if(mMotorIntake != null) {

            if(Math.abs(mSpeedIntake - VELOCITY_INTAKE) > 0.01) {
                mMotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mMotorIntake.setPower(VELOCITY_INTAKE);
                mSpeedIntake = VELOCITY_INTAKE;

                mLogger.info("Changing Speed");
            }

            mCurrentSpeedIntake = mMotorIntake.getVelocity();

            mLogger.metric("COMMAND_INTAKE",""+mSpeedIntake);
            mLogger.metric("VELOCITY_INTAKE", ""+mCurrentSpeedIntake);

        }

        if(mMotorOuttake != null) {

            if((Math.abs(mP.get() - mPCurrent) > 0.01) || (Math.abs(mI.get() - mICurrent) > 0.01) || (Math.abs(mD.get() - mDCurrent) > 0.01)|| (Math.abs(mF.get() - mFCurrent) > 0.01)) {

                mCoef = new PIDFCoefficients(mP.get(),mI.get(),mD.get(),mF.get());

                mPCurrent = mP.get();
                mICurrent = mI.get();
                mDCurrent = mD.get();
                mFCurrent = mF.get();

                mLogger.info("Changing PIDF");

            }

        }

        if(mServoOuttake != null) {

            if(Math.abs(mPositionOuttake - POSITION_OUTTAKE) > 0.01) {
                mServoOuttake.setPosition(POSITION_OUTTAKE);
                mPositionOuttake = POSITION_OUTTAKE;
                mLogger.info("Changing servo position");
            }

        }

        if(mFile != null) {
            try {
                mFile.append("" + System.currentTimeMillis() + "," + mSpeedOuttake + "," + mCurrentSpeedOuttake + "," + mPCurrent + "," + mICurrent + "," + mDCurrent + "," + mFCurrent + "," + mPositionOuttake + "\n");
            } catch (IOException e) {
                mLogger.warning(e.getMessage());
            }
        }

        mLogger.update();

    }

    @Override
    public void stop() {
        if(mFile != null) {
            try {
                mFile.flush();
                mFile.close();
            } catch (IOException e) {
                mLogger.warning(e.getMessage());
            }

        }
    }



}
