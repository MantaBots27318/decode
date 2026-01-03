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
import java.util.Map;

/* Qualcomm includes */
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.ServoCoupled;
import org.firstinspires.ftc.teamcode.components.ServoMock;
import org.firstinspires.ftc.teamcode.components.ServoSingle;

/*  Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

@Config
@TeleOp
public class ShootingTest extends OpMode {

    public static String MOTOR="outtake-wheels";
    public static double VELOCITY;
    public static String SERVO="outtake-lever-arm";
    public static double POSITION;

    MotorComponent  mMotor = null;
    ServoComponent  mServo = null;
    Logger          mLogger;

    double          mSpeed = 0;
    double          mPower = 0;
    double          mCurrentSpeed = 0;
    double          mPosition = 0;

    PIDFProvider    mP;
    PIDFProvider    mI;
    PIDFProvider    mD;
    PIDFProvider    mF;

    double          mPCurrent;
    double          mICurrent;
    double          mDCurrent;
    double          mFCurrent;

    FileWriter      mFile;

    @Override
    public void init() {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"speed-test");

        ConfMotor confm = Configuration.s_Current.getMotor(MOTOR);
        if(confm != null) {
            if (confm.shallMock()) { mMotor = new MotorMock(MOTOR); }
            else if (confm.getHw().size() == 1) { mMotor = new MotorSingle(confm, hardwareMap, MOTOR, mLogger); }
            else if (confm.getHw().size() == 2) { mMotor = new MotorCoupled(confm, hardwareMap, MOTOR, mLogger); }

            if(mMotor != null) {
                PIDFCoefficients initial = mMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                if(initial == null) initial = new PIDFCoefficients(0,0,0,0);

                mPCurrent = initial.p;
                mICurrent = initial.i;
                mDCurrent = initial.d;
                mFCurrent = initial.f;

                mP = new PIDFProvider(initial.p);
                mI = new PIDFProvider(initial.i);
                mD = new PIDFProvider(initial.d);
                mF = new PIDFProvider(initial.f);

                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"P",mP);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"I",mI);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"D",mD);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"F",mF);

                FtcDashboard.getInstance().updateConfig();
            }


        }

        String filepath = Environment.getExternalStorageDirectory().getPath()
                + "/FIRST/shooting-test.csv";
        mFile = null;
        try {
            mFile = new FileWriter(filepath);
            mFile.append("time,command,speed,power,p,i,d,f,position\n");
        } catch (IOException e) {
            mLogger.warning(e.getMessage());
        }

        if(confm == null) { mLogger.warning("Could not find motor named " + MOTOR + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mMotor== null) { mLogger.warning("Motor not initialized"); }

        ConfServo confs = Configuration.s_Current.getServo(SERVO);
        if(confs != null) {
            if (confs.shallMock()) { mServo = new ServoMock(SERVO); }
            else if (confs.getHw().size() == 1) { mServo = new ServoSingle(confs, hardwareMap, SERVO, mLogger); }
            else if (confs.getHw().size() == 2) { mServo = new ServoCoupled(confs, hardwareMap, SERVO, mLogger); }

        }

        if(confs == null) { mLogger.warning("Could not find servo named " + SERVO + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mServo== null) { mLogger.warning("Servo not initialized"); }

        mLogger.update();

    }

    @Override
    public void loop() {

        if(mMotor != null) {

            if(Math.abs(mSpeed - VELOCITY) > 0.01) {
                mMotor.setVelocity(VELOCITY * Math.PI / 180);
                mSpeed = VELOCITY;


                mLogger.info("Changing Speed");
            }

            mCurrentSpeed = mMotor.getVelocity()/ Math.PI * 180;
            mPower = mMotor.getPower();

            mLogger.metric("COMMAND",""+mSpeed);
            mLogger.metric("VELOCITY", ""+mCurrentSpeed);
            mLogger.metric("POWER", ""+mPower);

        }

        if(mMotor != null) {

            if((Math.abs(mP.get() - mPCurrent) > 0.01) || (Math.abs(mI.get() - mICurrent) > 0.01) || (Math.abs(mD.get() - mDCurrent) > 0.01)|| (Math.abs(mF.get() - mFCurrent) > 0.01)) {

                PIDFCoefficients coef = new PIDFCoefficients(mP.get(),mI.get(),mD.get(),mF.get());
                mMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coef);

                mPCurrent = mP.get();
                mICurrent = mI.get();
                mDCurrent = mD.get();
                mFCurrent = mF.get();

                mLogger.info("Changing PIDF");

            }

        }

        if(mServo != null) {

            if(Math.abs(mPosition - POSITION) > 0.01) {
                mServo.setPosition(POSITION);
                mPosition = POSITION;
            }

        }

        if(mFile != null) {
            try {
                mFile.append("" + System.currentTimeMillis() + "," + mSpeed + "," + mCurrentSpeed + "," + mPower + "," + mPCurrent + "," + mICurrent + "," + mDCurrent + "," + mFCurrent + "," + mPosition + "\n");
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


    static class PIDFProvider implements ValueProvider<Double> {
        Double mValue;
        public PIDFProvider( double Value) {
            mValue = Value;
        }
        @Override
        public Double get()              { return mValue;  }
        @Override
        public void set(Double Value)    { mValue = Value; }
    }

}
