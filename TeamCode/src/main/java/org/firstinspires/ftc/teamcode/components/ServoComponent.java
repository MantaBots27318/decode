/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   ServoComponent is the abstract interface managing servos,
   mocked servos and coupled servos with the same methods
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.utils.Logger;

import java.util.Map;

public abstract class ServoComponent {

    protected boolean  mReady;
    protected String   mName;


    public static ServoComponent factory(ConfServo config, HardwareMap hwm, String name, Logger logger) {

        ServoComponent result = null;
        // Build motor based on configuration
        if (config.shallMock()) { result = new ServoMock(name); }
        else if (config.getHw().size() == 1) { result = new ServoSingle(config, hwm, name, logger); }
        else if (config.getHw().size() == 2) { result = new ServoCoupled(config, hwm, name, logger); }

        return result;
    }

    public boolean                      isReady() { return mReady;}
    public String                       getName() { return mName; }

    /* ---------- Servo methods override --------- */

    public abstract Servo.Direction     getDirection();
    public abstract double	            getPosition();
    public abstract void	            scaleRange(double min, double max);

    public abstract void	            setDirection(Servo.Direction direction);
    public abstract void	            setPosition(double position);
    public abstract void	            reset();

}
