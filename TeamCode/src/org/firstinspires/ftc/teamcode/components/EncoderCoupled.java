/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   MotorCoupled class overloads the FTC motor class to manage
   A couple of motors both turning the same hardware.

   Note that this is a dangerous situation which can result in
   motor destruction if not correctly tuned. The coupled motors
   shall be the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* System includes */
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.acmerobotics.roadrunner.ftc.Encoder;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.utils.Logger;


public class EncoderCoupled extends EncoderComponent {

    Logger                          mLogger;

    int                             mFirstInvertPosition;
    int                             mSecondInvertPosition;

    Encoder                         mFirst;
    Encoder                         mSecond;

    /* -------------- Constructors --------------- */
    public EncoderCoupled(ConfEncoder conf, HardwareMap hwMap, String name, Logger logger)
    {
        mReady  = true;
        mLogger = logger;
        mName   = name;
        mFirstInvertPosition = 1;
        mSecondInvertPosition = 1;

        Map<String, Boolean> hw = conf.getHw();
        Map<String, Boolean> directions = conf.getDirection();
        if((hw.size() == 2) && (directions.size() == 2) && !conf.shallMock()) {

            List<Map.Entry<String, Boolean>> encoders = new ArrayList<>(hw.entrySet());
            List<Map.Entry<String, Boolean>> inverts = new ArrayList<>(directions.entrySet());
            ListIterator<Map.Entry<String, Boolean>> hwiterator = encoders.listIterator();
            ListIterator<Map.Entry<String, Boolean>> inviterator = inverts.listIterator();

            Map.Entry<String,Boolean> encoder = hwiterator.next();
            Map.Entry<String,Boolean> invert = inviterator.next();
            try {
                mFirst = new OverflowEncoder(new RawEncoder(hwMap.get(DcMotorEx.class, encoder.getKey())));
            }
            catch(Exception e) { mFirst = null; }
            if(mFirst != null && encoder.getValue()) { mFirst.setDirection(DcMotor.Direction.REVERSE);}
            else if(mFirst != null)                  { mFirst.setDirection(DcMotor.Direction.FORWARD);}

            if(invert.getValue()) { mFirstInvertPosition = -1; }

            encoder = hwiterator.next();
            invert = inviterator.next();
            try {
                mSecond = new OverflowEncoder(new RawEncoder(hwMap.get(DcMotorEx.class, encoder.getKey())));
            }
            catch(Exception e) { mSecond = null; }
            if(mSecond != null && encoder.getValue()) { mSecond.setDirection(DcMotor.Direction.REVERSE);}
            else if(mSecond != null)                  { mSecond.setDirection(DcMotor.Direction.FORWARD);}

            if(invert.getValue()) { mSecondInvertPosition = -1; }

        }

        if(mFirst  == null) { mReady = false; }
        if(mSecond == null) { mReady = false; }
    }

    @Override
    public int	                        getCurrentPosition()
    {
        int result = -1;
        if(mReady) {
            result = (int) (0.5 * mFirstInvertPosition * mFirst.getPositionAndVelocity().position +
                    mSecondInvertPosition * 0.5 * mSecond.getPositionAndVelocity().position);
        }
        return result;
    }

    @Override
    public double                       getVelocity()
    {
        double result = 0;
        if(mReady) {
            result = 0.5 * mSecond.getPositionAndVelocity().velocity + 0.5 * mFirst.getPositionAndVelocity().velocity;
        }
        return result;

    }

    @Override
    public String                      logPositions()
    {
        String result = "";
        if(mReady) {
            result += "\n  First : P : " + mFirst.getPositionAndVelocity().position + " V : " + mFirst.getPositionAndVelocity().velocity ;
            result += "\n  Second : P : " + mSecond.getPositionAndVelocity().position + " V : " + mSecond.getPositionAndVelocity().velocity ;
        }
        return result;
    }

}
