package org.firstinspires.ftc.teamcode.intake;


/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configurations includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

public class IntakeBrushes {

    Telemetry               mLogger;      // Local logger

    boolean                 mReady;       // True if component is able to fulfil its mission
    boolean                 mIsMoving;

    boolean                 mIsReversed;

    MotorComponent          mMotor;       // Motor rotating the wheels

    // Check if the component is currently moving on command
    public boolean isMoving() { return mIsMoving; }

    public boolean isReversed() { return mIsReversed; }

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;
        mIsMoving = false;
        mIsReversed = false;

        String status = "";

        ConfMotor intake = config.getMotor("intake-brushes");
        if(intake == null)  { mReady = false; status += " CONF";}
        else {

            // Build motor based on configuration
            if (intake.shallMock()) { mMotor = new MotorMock("intake-brushes"); }
            else if (intake.getHw().size() == 1) { mMotor = new MotorSingle(intake, hwm, "intake-brushes", logger); }
            else if (intake.getHw().size() == 2) { mMotor = new MotorCoupled(intake, hwm, "intake-brushes", logger); }

            if (!mMotor.isReady()) { mReady = false; status += " HW";}
            else {
                // Initialize motor
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }

        // Log status
        if (mReady) { logger.addLine("==>  IN BRS : OK"); }
        else        { logger.addLine("==>  IN BRS : KO : " + status); }


    }

    // Start the brushes with a given power
    public void start(double power)   {

        if(mReady)
        {
            mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mMotor.setPower(power);
            mIsMoving = true;
            if(power < 0) { mIsReversed = true; }
            else { mIsReversed = false; }
        }

    }

    // Stop brushes
    public void stop() {
        if(mReady) {
            mMotor.setPower(0);
            mIsMoving = false;
            mIsReversed = false;
        }
    }

    public void persist(Configuration config) {}

}