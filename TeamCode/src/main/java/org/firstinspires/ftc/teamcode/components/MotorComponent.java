/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   MotorComponent is the abstract interface managing motors,
   mocked motors and coupled motors with the same methods
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.utils.Logger;

public abstract class MotorComponent {

    protected boolean  mReady;
    protected String   mName;

    public static MotorComponent factory(ConfMotor config, HardwareMap hwm, String name, Logger logger) {

        MotorComponent result = null;

        // Build motor based on configuration
        if (config.shallMock()) { result = new MotorMock(name); }
        else if (config.getHw().size() == 1) { result = new MotorSingle(config, hwm, name, logger); }
        else if (config.getHw().size() == 2) { result = new MotorCoupled(config, hwm, name, logger); }

        // Default configuration
        if (result.isReady()) {
            // Initialize motor
            result.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            result.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        return result;
    }


    public abstract String                      logPositions();

    /* --------------------- DcMotor functions --------------------- */

    public boolean                              isReady() { return mReady;}
    public String                               getName() { return mName; }
    public abstract boolean	                    isBusy();

    public abstract int	                        getCurrentPosition();
    public abstract DcMotor.RunMode	            getMode();
    public abstract int	                        getTargetPosition();
    public abstract DcMotorSimple.Direction     getDirection();
    public abstract DcMotor.ZeroPowerBehavior	getZeroPowerBehavior();
    public abstract double                      getPower();

    public abstract void	                    setMode(DcMotor.RunMode mode);
    public abstract void	                    setDirection(DcMotorSimple.Direction direction);
    public abstract void	                    setTargetPosition(int position);
    public abstract void	                    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);
    public abstract void                        setPower(double power);

    /* -------------------- DcMotorEx functions -------------------- */

    public abstract void                        setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients);
    public abstract PIDFCoefficients            getPIDFCoefficients(DcMotor.RunMode mode);
    public abstract void                        setTargetPositionTolerance(int tolerance);
    public abstract int                         getTargetPositionTolerance();
    public abstract double                      getVelocity();
    public abstract void                        setVelocity( double rate );

}
