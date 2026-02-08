/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   ServoSingle class overloads the FTC motor class to manage
   a single motor with the same functions as a couple of them
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.components;

/* System includes */
import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Map;
import java.util.List;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class ServoSingle extends ServoComponent {

    Logger                  mLogger;

    Servo.Direction         mDirection;

    Servo                   mServo;

    /* -------------- Constructors --------------- */
    public ServoSingle(ConfServo conf, HardwareMap hwMap, String name, Logger logger)
    {
        mReady = true;
        mLogger = logger;
        mDirection = Servo.Direction.FORWARD;
        mName = name;

        Map<String, Boolean> hw = conf.getHw();
        if((hw.size() == 1) && !conf.shallMock()) {

            List<Map.Entry<String, Boolean>> servos = new ArrayList<>(hw.entrySet());
            ListIterator<Map.Entry<String, Boolean>> iterator = servos.listIterator();

            Map.Entry<String,Boolean> servo = iterator.next();
            mServo = hwMap.tryGet(Servo.class, servo.getKey());
            if(mServo != null && servo.getValue()) { mServo.setDirection(Servo.Direction.REVERSE);}
            else if(mServo != null)                { mServo.setDirection(Servo.Direction.FORWARD);}

        }

        if(mServo  == null) { mReady = false; }
    }

    @Override
    public Servo.Direction	            getDirection()
    {
        Servo.Direction result = Servo.Direction.FORWARD;
        if(mReady) {
            result = mServo.getDirection();
        }
        return result;
    }

    @Override
    public double	                    getPosition()
    {
        double result = -1;
        if(mReady) {
            result = mServo.getPosition();
        }
        return result;
    }

    @Override
    public void	                        scaleRange(double min, double max)
    {
        if(mReady) {
            mServo.scaleRange(min, max);
        }
    }

    @Override
    public void	                        setDirection(Servo.Direction direction)
    {
        if(mReady) {
            mServo.setDirection(direction);
        }
    }

    @Override
    public void	                        setPosition(double position)
    {
        if(mReady) {
            mServo.setPosition(position);
        }
    }
}
