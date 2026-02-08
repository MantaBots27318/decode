/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   ServoComponent is the abstract interface managing servos,
   mocked servos and coupled servos with the same methods
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ServoComponent {

    protected boolean  mReady;
    protected String   mName;

    public boolean                      isReady() { return mReady;}
    public String                       getName() { return mName; }

    /* ---------- Servo methods override --------- */

    public abstract Servo.Direction     getDirection();
    public abstract double	            getPosition();
    public abstract void	            scaleRange(double min, double max);

    public abstract void	            setDirection(Servo.Direction direction);
    public abstract void	            setPosition(double position);

}
