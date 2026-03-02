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
    
    public static final double X_GATE_INCHES_BLUE =                 -12;
    public static final double X_GATE_INCHES_RED =                  -12;
    public static final double Y_GATE_INCHES_BLUE =                 60;
    public static final double Y_GATE_INCHES_RED =                  -60;
    public static final double ANGLE_GATE_RADIANS_BLUE =           Math.PI / 3;
    public static final double ANGLE_GATE_RADIANS_RED =            -Math.PI / 3;
    

    Pose2d                  mStartGate    = new Pose2d(0,0,0);
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

            mEndGate = new Pose2d(
                    mStartGate.position.x,
                        mStartGate.position.y + Y_DELTA_INTAKE_INCHES_RED,
                        mStartGate.heading.toDouble());
            mBackGate = new Pose2d(
                    mStartGate.position.x,
                        mStartGate.position.y + 0.4 * Y_DELTA_INTAKE_INCHES_RED,
                        mStartGate.heading.toDouble());

        }

        if (alliance == Alliance.BLUE) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE, ANGLE_START_RADIANS_BLUE);

            mStartGate = new Pose2d(X_GATE_INCHES_BLUE,Y_GATE_INCHES_BLUE,ANGLE_GATE_RADIANS_BLUE);
            mEndGate = new Pose2d(
                            mStartGate.position.x,
                            mStartGate.position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                            mStartGate.heading.toDouble());
            mBackGate = new Pose2d(
                            mStartGate.position.x,
                            mStartGate.position.y + 0.4 * Y_DELTA_INTAKE_INCHES_BLUE,
                            mStartGate.heading.toDouble());
        }
    }

    public Pose2d   startGate()                    { return mStartGate; }
    public Pose2d   endGate()                      { return mEndGate; }
    public Pose2d   backGate()                     { return mBackGate; }

    public void log() {

        mLogger.info(Logger.Target.DRIVER_STATION,"START GATE X : " + mStartGate.position.x + " Y: " + mStartGate.position.y + " H: " + mStartGate.heading.toDouble());
        mLogger.info(Logger.Target.DRIVER_STATION,"END GATE X : " + mEndGate.position.x + " Y: " + mEndGate.position.y + " H: " + mEndGate.heading.toDouble());
        mLogger.info(Logger.Target.DRIVER_STATION,"BACK GATE X : " + mBackGate.position.x + " Y: " + mBackGate.position.y + " H: " + mBackGate.heading.toDouble());

        super.log();

    }

}
