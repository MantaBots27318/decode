/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Outtake lever arm subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* System includes */

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class Transfer {

    public enum Position {
        BLOCK,
        LET,
        DOWN
    }

    public enum State {
        NONE,
        WAITING,
        DOWN,
        LET,
        BLOCK
    }

    private static final Map<String, Position> sConfToPosition = Map.of(
            "let", Position.LET,
            "block", Position.BLOCK,
            "down", Position.DOWN
    );

    private static final int    sTimeOut = 100; // Timeout in ms

    Logger                      mLogger;      // Local logger

    boolean                     mReady;       // True if component is able to fulfil its mission
    SmartTimer                  mTimer;       // Timer for timeout management
    boolean                     mOpen;        // True if component is able to fulfil its mission
    boolean                     mOngoing;     // True if component is able to fulfil its mission

    Position                    mPosition;    // Current elbow position
    ServoComponent              mServo;       // Servos (coupled if specified by the configuration) driving the elbow
    Map<Position, Double>       mPositions;   // Link between positions enumerated and servos positions

    State                       mState;

    // Return current reference position
    public boolean isMoving() { return mTimer.isArmed();}

    // Check if the component is currently moving on command
    public Position getPosition() { return mPosition; }

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger = logger;
        mReady = true;
        mOngoing = false;
        mState = State.NONE;

        mPositions   = new LinkedHashMap<>();
        mTimer = new SmartTimer(mLogger);

        String status = "";

        // Get configuration
        ConfServo pitch  = config.getServo("transfer-servo");
        if(pitch == null)  { mReady = false; status += " CONF";}
        else {

            mServo = ServoComponent.factory(pitch,hwm, "transfer-servo", logger);

            mPositions.clear();
            Map<String, Double> confPosition = pitch.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }  else {
                    mLogger.info("Found unmanaged intake lever arm position : " + pos.getKey());
                }
            }

            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.info( "==>  IN ENTRY : OK"); }
        else        { logger.warning( "==>  IN ENTRY : KO : " + status); }

        // Initialize position
        this.setPosition(Position.BLOCK);
        mOpen = false;

    }

    // Make the servo reach current position. A timer is armed, and the servo won't respond until it is unarmed.
    // By the time, the servo should have reached its target position
    public void setPosition(Position position) {

        if( mPositions.containsKey(position) && mReady && !this.isMoving()) {
            mServo.setPosition(mPositions.get(position));
            mPosition = position;
            mTimer.arm(sTimeOut);
        }
    }
    public void setPosition(Position position, int timeout) {

        if( mPositions.containsKey(position) && mReady && !this.isMoving()) {
            mServo.setPosition(mPositions.get(position));
            mPosition = position;
            mTimer.arm(timeout);
        }

    }

    public void loop() {
        if (mState != State.NONE) { open_and_close_loop(); }
    }

    public boolean isOpen() { return mOpen; }
    public boolean ongoing() { return mOngoing; }

    public void close() {
        this.setPosition(Position.BLOCK);
        mOpen = false;
    }

    public void open() {
        this.setPosition(Position.LET);
        mOpen = true;
    }

    public void open_and_close_loop() {

        if (mState == State.NONE) {
            mState = State.WAITING;
        }
        else if (mState == State.WAITING) {
            setPosition(Transfer.Position.LET,2000);
            if (mPosition == Transfer.Position.LET)  {
                mState = State.LET;
            }
        }
        else if(mState == State.LET && !isMoving()) {
            setPosition(Transfer.Position.BLOCK);
            if (getPosition() == Transfer.Position.BLOCK)  {
                mState = State.BLOCK;
            }
        }
        else if (mState == State.BLOCK && !isMoving()) {
            mState = State.NONE;
        }

        mOngoing = mState != State.NONE;
    }



    public void open_down_and_close_loop() {

        if (mState == State.NONE) {
            mState = State.WAITING;
        }
        else if (mState == State.WAITING) {
            setPosition(Position.DOWN,100);
            if (mPosition == Transfer.Position.DOWN)  {
                mState = State.DOWN;
            }
        }
        else if (mState == State.DOWN) {
            setPosition(Transfer.Position.LET,2000);
            if (mPosition == Transfer.Position.LET)  {
                mState = State.LET;
            }
        }
        else if(mState == State.LET && !isMoving()) {
            setPosition(Transfer.Position.BLOCK);
            if (getPosition() == Transfer.Position.BLOCK)  {
                mState = State.BLOCK;
            }
        }
        else if (mState == State.BLOCK && !isMoving()) {
            mState = State.NONE;
        }

        mOngoing = mState != State.NONE;
    }

}