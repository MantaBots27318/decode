package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Robot include */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
public class ManualOpMode extends OpMode {

    Driving         mDriving;
    Collecting      mCollecting;
    Vision          mVision;
    @Override
    public void init(){

        try {
            mDriving = new Driving();
            mCollecting = new Collecting();
            mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);

            mVision.initialize();

            mDriving.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad1,mVision);
            mCollecting.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad2);
        }
        catch(Exception e){
            telemetry.addLine("INIT error : " + e.getMessage()) ;
        }

    }

    @Override
    public void loop (){


        try {
            mDriving.control();
            mCollecting.control();

            // Update state machines
            mCollecting.loop();
            telemetry.update();
            FtcDashboard.getInstance().getTelemetry().update();
        }
        catch(Exception e){
            telemetry.addLine("LOOP error : " + e.getMessage()) ;
        }

    }

    @Override
    public void stop() {
        // Make sure that once Teleop is over, we reset all the persisted data
        Configuration.s_Current.reinit();
        mVision.close();
    }


}