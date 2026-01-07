/* -------------------------------------------------------
   Copyright (c) [2026] FASNY
   All rights reserved
   -------------------------------------------------------
   Led test : opmode to test leds
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.test;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.LedComponent;
import org.firstinspires.ftc.teamcode.components.LedCoupled;
import org.firstinspires.ftc.teamcode.components.LedMock;
import org.firstinspires.ftc.teamcode.components.LedSingle;


/*  Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfLed;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

@Config
@TeleOp
public class LedTest extends OpMode {

    public static String    NAME="";
    public static String    COLOR = LedComponent.Color.NONE.text();

    LedComponent            mLed   = null;
    Logger                  mLogger;
    LedComponent.Color      mColor = LedComponent.Color.NONE;

    @Override
    public void init() {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"led-test");

        ConfLed conf = Configuration.s_Current.getLed(NAME);
        if(conf != null) {

            if (conf.shallMock()) { mLed = new LedMock(NAME); }
            else if (conf.getHw().size() == 1) { mLed = new LedSingle(conf, hardwareMap, NAME, mLogger); }
            else if (conf.getHw().size() == 2) { mLed = new LedCoupled(conf, hardwareMap, NAME, mLogger); }

        }

        if(conf == null)  { mLogger.warning("Could not find led named " + NAME + " in configuration " + Configuration.s_Current.getVersion()); }
        if(mLed== null) { mLogger.warning("Led not initialized"); }

        mLogger.update();

    }

    @Override
    public void loop() {

        if(mLed != null) {
            
            if(!COLOR.equals(mColor.text())) {
                if(COLOR.compareTo(LedComponent.Color.GREEN.text()) == 0) {
                    mLogger.info("vert");
                    mLed.on(LedComponent.Color.GREEN);
                    mColor = LedComponent.Color.GREEN;
                }
                if(COLOR.compareTo(LedComponent.Color.RED.text()) == 0) {
                    mLogger.info("rouge");
                    mLed.on(LedComponent.Color.RED);
                    mColor = LedComponent.Color.RED;
                }
                if(COLOR.compareTo(LedComponent.Color.NONE.text()) == 0) {
                    mLogger.info("off");
                    mLed.off();
                    mColor = LedComponent.Color.NONE;
                }
            }


            mLogger.update();

        }

    }

}
