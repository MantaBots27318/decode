/* -------------------------------------------------------
   Copyright (c) [2026] FASNY
   All rights reserved
   -------------------------------------------------------
   Speed test : opmode to test motor speed PID
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.test;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

/*  Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

@Config
@TeleOp
public class SpeedTest extends OpMode {

    public static String NAME="";
    public static double VELOCITY;

    MotorComponent  mMotor = null;
    Logger          mLogger;
    double          mSpeed = 0;

    @Override
    public void init() {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"speed-test");

        ConfMotor conf = Configuration.s_Current.getMotor(NAME);
        if(conf != null) {
            if (conf.shallMock()) { mMotor = new MotorMock(NAME); }
            else if (conf.getHw().size() == 1) { mMotor = new MotorSingle(conf, hardwareMap, NAME, mLogger); }
            else if (conf.getHw().size() == 2) { mMotor = new MotorCoupled(conf, hardwareMap, NAME, mLogger); }

        }

        if(conf == null)  { mLogger.warning("Could not find motor named " + NAME + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mMotor== null) { mLogger.warning("Motor not initialized"); }
        mLogger.update();

    }

    @Override
    public void loop() {

        if(mMotor != null) {

            if(Math.abs(mSpeed - VELOCITY) > 0.01) {
                mMotor.setVelocity(VELOCITY * Math.PI / 180);
                mSpeed = VELOCITY;
            }

            mLogger.metric("COMMAND",""+mSpeed);
            mLogger.metric("VELOCITY", ""+mMotor.getVelocity()/ Math.PI * 180);
            mLogger.metric("POWER", ""+mMotor.getPower());

            mLogger.update();
        }

    }

}
