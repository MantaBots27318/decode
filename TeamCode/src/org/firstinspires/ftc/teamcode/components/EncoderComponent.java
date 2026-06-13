/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   MotorComponent is the abstract interface managing motors,
   mocked motors and coupled motors with the same methods
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.utils.Logger;

public abstract class EncoderComponent {

    protected boolean  mReady;
    protected String   mName;

    public static EncoderComponent factory(ConfEncoder config, HardwareMap hwm, String name, Logger logger) {

        EncoderComponent result = null;

        // Build motor based on configuration
        if (config.shallMock()) { result = new EncoderMock(name); }
        else if (config.getHw().size() == 1) { result = new EncoderSingle(config, hwm, name, logger); }
        else if (config.getHw().size() == 2) { result = new EncoderCoupled(config, hwm, name, logger); }

        return result;
    }


    public abstract String                      logPositions();

    /* --------------------- DcMotor functions --------------------- */

    public boolean                              isReady() { return mReady;}
    public String                               getName() { return mName; }

    public abstract int	                        getCurrentPosition();
    public abstract double                      getVelocity();

}
