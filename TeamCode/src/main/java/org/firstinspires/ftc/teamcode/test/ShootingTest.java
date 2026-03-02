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

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Components includes */
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.EncoderComponent;

/*  Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

/* Utils includes */
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.pose.Posable;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
@TeleOp(name="ShootingTest", group="Test")
public class ShootingTest extends OpMode {

    public static String MOTOR_OUTTAKE="outtake-wheels";
    public static double VELOCITY_OUTTAKE;
    public static String MOTOR_INTAKE="intake-wheels";
    public static double POWER_INTAKE;
    public static String MOTOR_GUIDING="guiding-wheels";
    public static double POWER_GUIDING;
    public static String SERVO_TRANSFER="transfer-servo";
    public static double POSITION_TRANSFER = 0.58;
    public static String SERVO_HOOD="turret-hood";
    public static double POSITION_HOOD = 0.5;
    public static String SERVO_ROTATION="turret-rotation";
    public static double POSITION_ROTATION = 0.5;
    public static String ENCODER_ROTATION="turret-rotation";

    MotorComponent      mMotorOuttake = null;
    MotorComponent      mMotorIntake = null;
    MotorComponent      mMotorGuiding = null;
    ServoComponent      mServoRotation = null;
    ServoComponent      mServoHood = null;
    ServoComponent      mServoTransfer = null;
    EncoderComponent    mEncoderRotation = null;
    Logger              mLogger;
    double              mInitialEncoderPosition;

    double              mSpeedOuttake = 0;
    double              mPowerIntake = 0;
    double              mPowerGuiding = 0;
    double              mCurrentSpeedOuttake = 0;
    double              mPositionTransfer = 0;
    double              mPositionRotation = 0.5;
    double              mPositionHood = 0.5;
    double              mEncoderPosition = 0;
    double              mDistance;

    Path                mPath;

    Vision              mVision;

    PIDFController.PIDFProvider    mP;
    PIDFController.PIDFProvider    mI;
    PIDFController.PIDFProvider    mD;
    PIDFController.PIDFProvider    mF;
    PIDFCoefficients               mCoef;

    double              mPCurrent;
    double              mICurrent;
    double              mDCurrent;
    double              mFCurrent;

    FileWriter          mFile;

    @Override
    public void init() {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"speed-test");

        ConfMotor confmo = Configuration.s_Current.getMotor(MOTOR_OUTTAKE);
        if(confmo != null) {
            mMotorOuttake = MotorComponent.factory(confmo,hardwareMap,MOTOR_OUTTAKE,mLogger);
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

        confmo = Configuration.s_Current.getMotor(MOTOR_INTAKE);
        if(confmo != null) {
            mMotorIntake = MotorComponent.factory(confmo, hardwareMap, MOTOR_INTAKE, mLogger);
        }
        if(confmo == null) { mLogger.warning("Could not find motor named " + MOTOR_INTAKE + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mMotorIntake== null) { mLogger.warning("Motor intake not initialized"); }

        confmo = Configuration.s_Current.getMotor(MOTOR_GUIDING);
        if(confmo != null) {
            mMotorGuiding = MotorComponent.factory(confmo, hardwareMap, MOTOR_GUIDING, mLogger);
        }
        if(confmo == null) { mLogger.warning("Could not find motor named " + MOTOR_GUIDING + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mMotorGuiding== null) { mLogger.warning("Motor guiding not initialized"); }

        ConfLimelight confli = Configuration.s_Current.getLimelight("limelight");
        if(confli != null) {
            mVision = new Vision(confli,hardwareMap,"vision",mLogger);
            if(mVision != null) { mVision.initialize(); }
            mPath = new PathAutonomousGoal(mLogger);
            mPath.initialize(Alliance.BLUE);
        }
        if(confli == null) { mLogger.warning("Could not find limelight named limelight in configuration " + Configuration.s_Current.getVersion()); }
        if(mVision == null) { mLogger.warning("Vision not initialized"); }

        ConfServo confs = Configuration.s_Current.getServo(SERVO_TRANSFER);
        if(confs != null) {
            mServoTransfer = ServoComponent.factory(confs,hardwareMap,SERVO_TRANSFER,mLogger);
            if(mServoTransfer != null) { mServoTransfer.setPosition(POSITION_TRANSFER); }

        }
        if(confs == null) { mLogger.warning("Could not find servo named " + SERVO_TRANSFER + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mServoTransfer== null) { mLogger.warning("Servo transfer not initialized"); }

        confs = Configuration.s_Current.getServo(SERVO_HOOD);
        if(confs != null) {
            mServoHood = ServoComponent.factory(confs,hardwareMap,SERVO_HOOD,mLogger);
            if(mServoHood != null) { mServoHood.setPosition(POSITION_HOOD); }
        }
        if(confs == null) { mLogger.warning("Could not find servo named " + SERVO_HOOD + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mServoHood== null) { mLogger.warning("Servo transfer not initialized"); }

        confs = Configuration.s_Current.getServo(SERVO_ROTATION);
        if(confs != null) {
            mServoRotation = ServoComponent.factory(confs,hardwareMap,SERVO_ROTATION,mLogger);
            if(mServoRotation != null) { mServoRotation.setPosition(POSITION_ROTATION); }
        }
        if(confs == null) { mLogger.warning("Could not find servo named " + SERVO_ROTATION + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mServoRotation== null) { mLogger.warning("Servo transfer not initialized"); }

        ConfEncoder confe = Configuration.s_Current.getEncoder(ENCODER_ROTATION);
        if(confe != null) {
            mEncoderRotation = EncoderComponent.factory(confe,hardwareMap,ENCODER_ROTATION,mLogger);
            if(mEncoderRotation != null) { mInitialEncoderPosition = mEncoderRotation.getCurrentPosition(); }
        }
        if(confe == null) { mLogger.warning("Could not find encoder named " + ENCODER_ROTATION + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mEncoderRotation== null) { mLogger.warning("Encoder rotation not initialized"); }

        String filepath = Environment.getExternalStorageDirectory().getPath()
                + "/FIRST/shooting-test.csv";
        mFile = null;
        try {
            mFile = new FileWriter(filepath);
            mFile.append("time,outtake command,outtake speed,p,i,d,f,intake power, guiding power, transfer position, hood position, rotation position, encoder position, encoder, delta, distance\n");
        } catch (IOException e) {
            mLogger.warning(e.getMessage());
        }

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
                mLogger.info("Changing Outtake Speed");
            }

            mCurrentSpeedOuttake = mMotorOuttake.getVelocity();
            mLogger.metric("COMMAND_OUTTAKE",""+mSpeedOuttake);
            mLogger.metric("VELOCITY_OUTTAKE", ""+mCurrentSpeedOuttake);

        }

        if(mMotorIntake != null) {

            if(Math.abs(mPowerIntake - POWER_INTAKE) > 0.01) {
                mMotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mMotorIntake.setPower(POWER_INTAKE);
                mPowerIntake = POWER_INTAKE;

                mLogger.info("Changing Intake Power");
            }
            mLogger.metric("POWER_INTAKE",""+mPowerIntake);

        }

        if(mMotorGuiding != null) {

            if(Math.abs(mPowerGuiding - POWER_GUIDING) > 0.01) {
                mMotorGuiding.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mMotorGuiding.setPower(POWER_GUIDING);
                mPowerGuiding = POWER_GUIDING;
                mLogger.info("Changing Guiding Power");
            }
            mLogger.metric("POWER_GUIDING",""+mPowerGuiding);
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

        if(mServoTransfer != null) {

            if(Math.abs(mPositionTransfer - POSITION_TRANSFER) > 0.01) {
                mServoTransfer.setPosition(POSITION_TRANSFER);
                mPositionTransfer = POSITION_TRANSFER;
                mLogger.info("Changing transfer servo position");
            }
            mLogger.metric("POSITION_TRANSFER",""+mPositionTransfer);

        }

        if(mServoHood != null) {

            if(Math.abs(mPositionHood - POSITION_HOOD) > 0.01) {
                mServoHood.setPosition(POSITION_HOOD);
                mPositionHood = POSITION_HOOD;
                mLogger.info("Changing hood servo position");
            }
            mLogger.metric("POSITION_HOOD",""+mPositionHood);

        }

        if(mServoRotation != null) {

            if(Math.abs(mPositionRotation - POSITION_ROTATION) > 0.0001) {
                mServoRotation.setPosition(POSITION_ROTATION);
                mPositionRotation = POSITION_ROTATION;
                mLogger.info("Changing rotation servo position");
            }
            mLogger.metric("POSITION_ROTATION",""+mPositionRotation);

        }

        if(mEncoderRotation != null) {
            mEncoderPosition = mEncoderRotation.getCurrentPosition();
            mLogger.metric("ENCODER_POSITION",""+mEncoderPosition);
        }

        if(mVision != null) {
            Pose3D limelight = mVision.getPosition();
            if(limelight != null) {
                Pose2d ftc = new Pose2d(
                        -limelight.getPosition().x * Path.M_TO_INCHES,
                        -limelight.getPosition().y * Path.M_TO_INCHES,
                        (limelight.getOrientation().getYaw() + 180) * Math.PI / 180);
                mLogger.info(""+ftc.position + " " + ftc.heading.toDouble() / Math.PI * 180);
                ftc = Posable.derivePose(
                       ftc,
                        new Pose2d(6,0,0)
                );
                mDistance = Math.sqrt(
                        (mPath.target().position.x - ftc.position.x) *
                                (mPath.target().position.x - ftc.position.x) +
                                (mPath.target().position.y - ftc.position.y) *
                                        (mPath.target().position.y - ftc.position.y));
                mLogger.metric("DISTANCE", "" + mDistance);
            }



        }

        if(mFile != null) {
            try {
                mFile.append(String.valueOf(System.currentTimeMillis()))
                        .append(",").append(String.valueOf(mSpeedOuttake))
                        .append(",").append(String.valueOf(mCurrentSpeedOuttake))
                        .append(",").append(String.valueOf(mPCurrent))
                        .append(",").append(String.valueOf(mICurrent))
                        .append(",").append(String.valueOf(mDCurrent))
                        .append(",").append(String.valueOf(mFCurrent))
                        .append(",").append(String.valueOf(mPowerIntake))
                        .append(",").append(String.valueOf(mPowerGuiding))
                        .append(",").append(String.valueOf(mPositionTransfer))
                        .append(",").append(String.valueOf(mPositionHood))
                        .append(",").append(String.valueOf(mPositionRotation))
                        .append(",").append(String.valueOf(mEncoderPosition))
                        .append(",").append(String.valueOf(mEncoderPosition - mInitialEncoderPosition))
                        .append(",").append(String.valueOf(mDistance))
                        .append("\n");
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
