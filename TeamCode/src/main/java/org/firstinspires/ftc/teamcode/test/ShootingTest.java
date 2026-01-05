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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Acmerobotics includes */
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
import org.firstinspires.ftc.teamcode.utils.PIDFController;

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
    MotorComponent  mMotorIntake = null;
    ServoComponent  mServoOuttake = null;
    Logger          mLogger;

    double          mSpeedOuttake = 0;
    double          mSpeedIntake = 0;
    double          mCurrentSpeedOuttake = 0;
    double          mCurrentSpeedIntake = 0;
    double          mPositionOuttake = 0;

    PIDFController.PIDFProvider    mP;
    PIDFController.PIDFProvider    mI;
    PIDFController.PIDFProvider    mD;
    PIDFController.PIDFProvider    mF;

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
                PIDFCoefficients initial = mMotorOuttake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                if(initial == null) initial = new PIDFCoefficients(200,3,0,0);
                mMotorOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,initial);

                mPCurrent = initial.p;
                mICurrent = initial.i;
                mDCurrent = initial.d;
                mFCurrent = initial.f;

                mP = new PIDFController.PIDFProvider(initial.p);
                mI = new PIDFController.PIDFProvider(initial.i);
                mD = new PIDFController.PIDFProvider(initial.d);
                mF = new PIDFController.PIDFProvider(initial.f);

                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"P",mP);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"I",mI);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"D",mD);
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"F",mF);

                FtcDashboard.getInstance().updateConfig();
            }


        }

        if(confmo == null) { mLogger.warning("Could not find motor named " + MOTOR_OUTTAKE + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mMotorOuttake== null) { mLogger.warning("Motor outtake not initialized"); }


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
                mMotorOuttake.setVelocity(VELOCITY_OUTTAKE);
                mSpeedOuttake = VELOCITY_OUTTAKE;


                mLogger.info("Changing Speed");
            }

            mCurrentSpeedOuttake = mMotorOuttake.getVelocity();

            mLogger.metric("COMMAND_OUTTAKE",""+mSpeedOuttake);
            mLogger.metric("VELOCITY_OUTTAKE", ""+mCurrentSpeedOuttake);

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

                PIDFCoefficients coef = new PIDFCoefficients(mP.get(),mI.get(),mD.get(),mF.get());
                mMotorOuttake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coef);

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
