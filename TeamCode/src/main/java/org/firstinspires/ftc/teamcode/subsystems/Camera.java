/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Class managing Camera servo
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.ServoCoupled;
import org.firstinspires.ftc.teamcode.components.ServoMock;
import org.firstinspires.ftc.teamcode.components.ServoSingle;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.utils.Logger;


public class Camera {

    public enum Position {
        TAG,
        BALL
    }

    private static final Map<String, Position> sConfToPosition = Map.of(
            "tag", Position.TAG,
            "ball", Position.BALL
    );

    private static final int    sTimeOut = 1000; // Timeout in ms

    Logger                      mLogger;      // Local logger

    boolean                     mReady;       // True if component is able to fulfil its mission
    SmartTimer                  mTimer;       // Timer for timeout management

    Position                    mPosition;    // Current elbow position
    ServoComponent              mServo;       // Servos (coupled if specified by the configuration) driving the elbow
    Map<Position, Double>       mPositions;   // Link between positions enumerated and servos positions

    // Return current reference position
    public boolean isMoving() { return mTimer.isArmed();}

    // Check if the component is currently moving on command
    public Position getPosition() { return mPosition; }

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger = logger;
        mReady = true;

        mPositions   = new LinkedHashMap<>();
        mTimer = new SmartTimer(mLogger);

        String status = "";

        // Get configuration
        ConfServo pitch  = config.getServo("camera");
        if(pitch == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (pitch.shallMock()) { mServo = new ServoMock("camera"); }
            else if (pitch.getHw().size() == 1) { mServo = new ServoSingle(pitch, hwm, "camera", logger); }
            else if (pitch.getHw().size() == 2) { mServo = new ServoCoupled(pitch, hwm, "camera", logger); }

            mPositions.clear();
            Map<String, Double> confPosition = pitch.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }  else {
                    mLogger.info("Found unmanaged camera position : " + pos.getKey());
                }
            }

            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.info("==>  OUT CAM : OK"); }
        else        { logger.warning("==>  OUT CAM : KO : " + status); }

        // Initialize position
        this.setPosition(Position.TAG);

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

}