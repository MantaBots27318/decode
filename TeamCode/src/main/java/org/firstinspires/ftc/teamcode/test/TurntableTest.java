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
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.components.EncoderComponent;
import org.firstinspires.ftc.teamcode.components.LedComponent;
import org.firstinspires.ftc.teamcode.components.LedCoupled;
import org.firstinspires.ftc.teamcode.components.LedMock;
import org.firstinspires.ftc.teamcode.components.LedSingle;
import org.firstinspires.ftc.teamcode.components.EncoderComponent;
import org.firstinspires.ftc.teamcode.components.EncoderCoupled;
import org.firstinspires.ftc.teamcode.components.EncoderMock;
import org.firstinspires.ftc.teamcode.components.EncoderSingle;
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.ServoCoupled;
import org.firstinspires.ftc.teamcode.components.ServoMock;
import org.firstinspires.ftc.teamcode.components.ServoSingle;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.configurations.ConfLed;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.LockQRCode;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.io.FileWriter;
import java.io.IOException;

@Config
@TeleOp(name="TurntableTest", group="Test")
public class TurntableTest extends OpMode {

    EncoderComponent mEncoder;
    ServoComponent      mServo;

    Logger              mLogger;

    public static double mPosition;

    @Override
    public void init() {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"turntable-test");

        ConfEncoder confenc = Configuration.s_Current.getEncoder("turret-rotation");
        if(confenc != null) {
            mEncoder = EncoderComponent.factory(confenc, hardwareMap, "turret-rotation", mLogger);
        }

        ConfServo confservo = Configuration.s_Current.getServo("turret-rotation");
        if(confservo != null) {
            mServo = ServoComponent.factory(confservo, hardwareMap, "turret-rotation", mLogger);
        }

        if(confenc == null) { mLogger.warning("Could not find encoder named turret-rotation in configuration " + Configuration.s_Current.getVersion()); }
        if(confservo == null) { mLogger.warning("Could not find servo named turret-rotation in configuration " + Configuration.s_Current.getVersion()); }

        mLogger.update();

    }

    @Override
    public void loop() {

        if(mServo != null && mEncoder != null) {
            mServo.setPosition(mPosition);
            mLogger.info("encoder value"+mEncoder.getCurrentPosition());
        }


        mLogger.update();

    }

    @Override
    public void stop() {

    }



}
