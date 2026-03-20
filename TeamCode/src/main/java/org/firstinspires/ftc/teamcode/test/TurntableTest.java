/* -------------------------------------------------------
   Copyright (c) [2026] FASNY
   All rights reserved
   -------------------------------------------------------
   Speed test : opmode to test motor speed PID
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.test;

/* Java includes */

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.EncoderComponent;
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;


@Config
@TeleOp(name="TurntableTest", group="Test")
public class TurntableTest extends OpMode {



    EncoderComponent mEncoder;
    ServoComponent      mServo;

    Logger              mLogger;

    double              mInitialEncoderPosition;

    double              mInitialServoPosition;
    SmartTimer          mTimer;

    public static double mPosition;
    public static double sEncoderAmplitude = 22315;

    public static boolean mShallReset = false;


    @Override
    public void init() {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"turntable-test");
        mTimer          = new SmartTimer(mLogger);

        ConfEncoder confenc = Configuration.s_Current.getEncoder("turret-rotation");
        if(confenc != null) {
            mEncoder = EncoderComponent.factory(confenc, hardwareMap, "turret-rotation", mLogger);
        }

        ConfServo confservo = Configuration.s_Current.getServo("turret-rotation");
        if(confservo != null) {
            mServo = ServoComponent.factory(confservo, hardwareMap, "turret-rotation", mLogger);
        }

        if(confenc == null) { mLogger.warning("Could not find encoder named turret-rotation in configuration " + Configuration.s_Current.getVersion()); }
        else if(confservo == null) { mLogger.warning("Could not find servo named turret-rotation in configuration " + Configuration.s_Current.getVersion()); }
        else {
            mServo.setPosition(mPosition);
            mTimer.arm(5000);
            mInitialEncoderPosition = mEncoder.getCurrentPosition();
            mInitialServoPosition = mServo.getPosition();
        }

        mLogger.update();

    }

    @Override
    public void loop() {

        if(mServo != null && mEncoder != null) {
            mServo.setPosition(mPosition);
            double enc = mEncoder.getCurrentPosition();
            double delta = enc-mInitialEncoderPosition;
            mLogger.info("encoder value : "+enc);
            mLogger.info("encoder delta : "+delta);
            mLogger.info("servo estimated value" + (delta / sEncoderAmplitude + mInitialServoPosition));
            if(mShallReset) {
                mInitialEncoderPosition = enc;
                mInitialServoPosition = mServo.getPosition();
            }
        }


        mLogger.update();

    }

}
