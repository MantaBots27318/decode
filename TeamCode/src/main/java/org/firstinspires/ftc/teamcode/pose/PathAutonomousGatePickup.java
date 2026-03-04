/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Class managing path reference positions for autonomous
   starting at the goal
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.pose;

/* ACME robotics includes /*/

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class PathAutonomousGatePickup extends PathAutonomousGoal {
    
    public static final double X_GATE_INCHES_BLUE =                 -11;
    public static final double X_GATE_INCHES_RED =                  -11;
    public static final double Y_GATE_INCHES_BLUE =                 53;
    public static final double Y_GATE_INCHES_RED =                  -53;
    public static final double ANGLE_GATE_RADIANS_BLUE =           68.0 / 180 * Math.PI;
    public static final double ANGLE_GATE_RADIANS_RED =            -68.0 / 180 * Math.PI;

    public static final double X_GATE_INTAKE_INCHES_BLUE =                 -22;
    public static final double X_GATE_INTAKE_INCHES_RED =                  -22;
    public static final double Y_GATE_INTAKE_INCHES_BLUE =                 59;
    public static final double Y_GATE_INTAKE_INCHES_RED =                  -59;
    public static final double ANGLE_GATE_INTAKE_RADIANS_BLUE =           27.0 / 180 * Math.PI;
    public static final double ANGLE_GATE_INTAKE_RADIANS_RED =            -27.0 / 180 * Math.PI;



    public static final double GATE_OPEN_DISTANCE = 8;
    

    Pose2d                  mStartGate    = new Pose2d(0,0,0);

    Pose2d                  mIntakeGate   = new Pose2d(0,0,0);
    Pose2d                  mEndGate      = new Pose2d(0,0,0);
    Pose2d                  mBackGate     = new Pose2d(0,0,0);


    public PathAutonomousGatePickup(Logger logger) {
        super(logger);
    }

    public void initialize(Alliance alliance) {

        super.initialize(alliance);

        if (alliance == Alliance.RED) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS_RED);

            mStartGate = new Pose2d(X_GATE_INCHES_RED,Y_GATE_INCHES_RED,ANGLE_GATE_RADIANS_RED);

            mIntakeGate = new Pose2d(X_GATE_INTAKE_INCHES_RED,Y_GATE_INTAKE_INCHES_RED,ANGLE_GATE_INTAKE_RADIANS_RED);

            mEndGate = new Pose2d(
                    mStartGate.position.x + GATE_OPEN_DISTANCE * Math.sin(ANGLE_GATE_RADIANS_RED),
                    mStartGate.position.y + GATE_OPEN_DISTANCE * Math.cos(ANGLE_GATE_RADIANS_RED),
                    mStartGate.heading.toDouble());

            mBackGate = new Pose2d(
                    mStartGate.position.x - GATE_OPEN_DISTANCE * Math.sin(ANGLE_GATE_RADIANS_RED),
                    mStartGate.position.y - GATE_OPEN_DISTANCE * Math.cos(ANGLE_GATE_RADIANS_RED),
                    mStartGate.heading.toDouble());
        }

        if (alliance == Alliance.BLUE) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE, ANGLE_START_RADIANS_BLUE);

            mIntakeGate = new Pose2d(X_GATE_INTAKE_INCHES_BLUE,Y_GATE_INTAKE_INCHES_BLUE,ANGLE_GATE_INTAKE_RADIANS_BLUE);

            mStartGate = new Pose2d(X_GATE_INCHES_BLUE,Y_GATE_INCHES_BLUE,ANGLE_GATE_RADIANS_BLUE);

            mEndGate = new Pose2d(
                    mStartGate.position.x + GATE_OPEN_DISTANCE * Math.sin(ANGLE_GATE_RADIANS_BLUE),
                    mStartGate.position.y + GATE_OPEN_DISTANCE * Math.cos(ANGLE_GATE_RADIANS_BLUE),
                    mStartGate.heading.toDouble());

            mBackGate = new Pose2d(
                    mStartGate.position.x - GATE_OPEN_DISTANCE * Math.sin(ANGLE_GATE_RADIANS_BLUE),
                    mStartGate.position.y - GATE_OPEN_DISTANCE * Math.cos(ANGLE_GATE_RADIANS_BLUE),
                    mStartGate.heading.toDouble());

        }
    }

    public Pose2d   startGate()                    { return mStartGate; }
    public Pose2d   endGate()                      { return mEndGate; }
    public Pose2d   intakeGate()                   { return mIntakeGate; }
    public Pose2d   backGate()                     { return mBackGate; }

    public void log() {

        mLogger.info(Logger.Target.DRIVER_STATION,"START GATE X : " + mStartGate.position.x + " Y: " + mStartGate.position.y + " H: " + mStartGate.heading.toDouble());
        mLogger.info(Logger.Target.DRIVER_STATION,"END GATE X : " + mEndGate.position.x + " Y: " + mEndGate.position.y + " H: " + mEndGate.heading.toDouble());
        mLogger.info(Logger.Target.DRIVER_STATION,"INTAKE GATE X : " + mIntakeGate.position.x + " Y: " + mIntakeGate.position.y + " H: " + mIntakeGate.heading.toDouble());
        mLogger.info(Logger.Target.DRIVER_STATION,"BACK GATE X : " + mBackGate.position.x + " Y: " + mBackGate.position.y + " H: " + mBackGate.heading.toDouble());

        super.log();

    }

}
