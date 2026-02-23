/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   MotorSingle class overloads the FTC motor class to manage
   a single motor with the same functions as a couple of them
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* System includes */

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.utils.Logger;

import com.acmerobotics.roadrunner.ftc.Encoder;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;

public class EncoderSingle extends EncoderComponent {

    Logger                          mLogger;

    Encoder                         mEncoder;

    int                             mInvertPosition;

    /* -------------- Constructors --------------- */
    public EncoderSingle(ConfEncoder conf, HardwareMap hwMap, String name, Logger logger)
    {
        mReady          = true;
        mLogger         = logger;
        mName           = name;
        mInvertPosition = 1;

        Map<String, Boolean> hw = conf.getHw();
        Map<String, Boolean> directions = conf.getDirection();
        if((hw.size() == 1) && (directions.size() == 1) && !conf.shallMock()) {

            List<Map.Entry<String, Boolean>> encoders = new ArrayList<>(hw.entrySet());
            List<Map.Entry<String, Boolean>> inverts = new ArrayList<>(directions.entrySet());
            ListIterator<Map.Entry<String, Boolean>> hwiterator = encoders.listIterator();
            ListIterator<Map.Entry<String, Boolean>> inviterator = inverts.listIterator();

            Map.Entry<String,Boolean> encoder = hwiterator.next();
            Map.Entry<String,Boolean> invert = inviterator.next();
            try { mEncoder = new OverflowEncoder(new RawEncoder(hwMap.get(DcMotorEx.class, encoder.getKey() ))); }
            catch(Exception e) { mEncoder = null; }
            if(mEncoder != null && encoder.getValue()) { mEncoder.setDirection(DcMotor.Direction.REVERSE);}
            else if(mEncoder != null)                  { mEncoder.setDirection(DcMotor.Direction.FORWARD);}

            if(invert.getValue()) { mInvertPosition = -1; }

        }

        if(mEncoder  == null) { mReady = false; }
    }

    @Override
    public int	                        getCurrentPosition()
    {
        int result = -1;
        if(mReady) {
            result = mInvertPosition * mEncoder.getPositionAndVelocity().position;
        }
        return result;
    }

    @Override
    public double                      getVelocity()
    {
        double result = 0;
        if(mReady) {
            result = mEncoder.getPositionAndVelocity().velocity;
        }
        return result;

    }

    @Override
    public String                      logPositions()
    {
        String result = "";
        if(mReady) {
            result += "\n  First : P : " + mEncoder.getPositionAndVelocity().position + " V : " + mEncoder.getPositionAndVelocity().velocity;
        }
        return result;
    }

}
