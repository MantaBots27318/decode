/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   MotorMock Enables mocking of a motor for testing purpose
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderMock extends EncoderComponent {

    int                         mPosition;

    public EncoderMock(String name)
    {
        mName = name;

    }

    @Override
    public int	                        getCurrentPosition() { return mPosition; }

    @Override
    public String                       logPositions() { return "" + this.getCurrentPosition(); }

    @Override
    public double                       getVelocity()
    {
        return 0.0;
    }

}
