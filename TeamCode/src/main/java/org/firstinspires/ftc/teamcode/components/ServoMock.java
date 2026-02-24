/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   ServoMock Enables mocking of a motor for testing purpose
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoMock extends ServoComponent {

    Servo.Direction     mDirection;
    double              mPosition;
    double              mMin;
    double              mMax;

    /* -------------- Constructors --------------- */
    public ServoMock(String name)
    {
        mName = name;
        mDirection = Servo.Direction.FORWARD;
    }

    /* ---------- Servo methods override --------- */

    @Override
    public Servo.Direction	            getDirection()  { return mDirection;  }

    @Override
    public double	                    getPosition()   { return mPosition;   }

    @Override
    public void	                        scaleRange(double min, double max)
    {
        mMin = min;
        mMax = max;
    }

    @Override
    public void	                        setDirection(Servo.Direction direction) { mDirection = direction; }

    @Override
    public void	                        setPosition(double position)
    {
        mPosition = min(position,mMax);
        mPosition = max(mPosition,mMin);
    }

    public void                         reset() {}

}
