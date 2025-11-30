package org.firstinspires.ftc.teamcode.outtake;


/* Qualcomm includes */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

public class OuttakeWheels {

    private static final int    sTimeOut = 5000; // Timeout in ms

    Telemetry                   mLogger;      // Local logger

    boolean                     mReady;       // True if component is able to fulfil its mission
    boolean                     mIsMoving;

    SmartTimer                  mTimer;       // Timer for timeout management

    MotorComponent              mMotor;       // Motor rotating the wheels

    // Check if the component is currently moving on command

    public boolean isTransitioning() { return mTimer.isArmed();}
    public boolean isMoving()        { return mIsMoving;}
    
    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;
        mIsMoving = false;

        mTimer = new SmartTimer(mLogger);

        String status = "";

        ConfMotor wheels = config.getMotor("outtake-wheels");
        if(wheels == null)  { mReady = false; status += " CONF";}
        else {

            // Build motor based on configuration
            if (wheels.shallMock()) { mMotor = new MotorMock("outtake-wheels"); }
            else if (wheels.getHw().size() == 1) { mMotor = new MotorSingle(wheels, hwm, "outtake-wheels", logger); }
            else if (wheels.getHw().size() == 2) { mMotor = new MotorCoupled(wheels, hwm, "outtake-wheels", logger); }

            if (!mMotor.isReady()) { mReady = false; status += " HW";}
            else {
                // Initialize motor
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }

        // Log status
        if (mReady) { logger.addLine("==>  OUT WHEELS : OK"); }
        else        { logger.addLine("==>  OUT WHEELS : KO : " + status); }

    }

    // Start the brushes with a given power
    public void start(double Power)   {

        if(mReady && !this.isTransitioning())
        {
            mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mMotor.setPower(Power);
            mIsMoving = true;
            mTimer.arm(sTimeOut);
        }

    }

    // Stop brushes
    public void stop() {
        if(mReady) {
            mMotor.setPower(0);
            mIsMoving = false;
            mTimer.arm(sTimeOut);
        }
    }

    public void persist(Configuration config) {}

}