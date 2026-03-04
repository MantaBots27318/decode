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

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Pattern;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

public class PathAutonomousGoal extends Path {

    public static final double X_START_INCHES =                     50;
    public static final double Y_START_INCHES_BLUE =                50;
    public static final double Y_START_INCHES_RED =                 - 50;
    public static final double ANGLE_START_RADIANS_RED =            - Math.PI / 180 * 50;
    public static final double ANGLE_START_RADIANS_BLUE =           Math.PI / 180 * 50;


    protected static final double Y_DELTA_INTAKE_INCHES_BLUE =      22;
    protected static final double Y_DELTA_INTAKE_INCHES_RED =       -22;

    public static final double X_GPP_PATTERN_INCHES_BLUE =          -36.25;
    public static final double X_PGP_PATTERN_INCHES_BLUE =          -12;
    public static final double X_PPG_PATTERN_INCHES_BLUE =          11.25;
    public static final double X_GPP_PATTERN_INCHES_RED =           -36.25;
    public static final double X_PGP_PATTERN_INCHES_RED =           -12;
    public static final double X_PPG_PATTERN_INCHES_RED =           11.25;


    public static final double Y_PATTERN_INCHES_BLUE =              30;
    public static final double Y_PATTERN_INCHES_RED =               -30;
    public static final double ANGLE_PATTERN_RADIANS_BLUE =         Math.PI / 2;
    public static final double ANGLE_PATTERN_RADIANS_RED =          -Math.PI / 2;

    public static final double TGT_INTAKE_TO_SHOOT_RADIANS_BLUE =   -Math.PI/4;
    public static final double TGT_INTAKE_TO_SHOOT_RADIANS_RED =    Math.PI/4;


    private static final double X_LEAVE_INCHES                     = 40;
    private static final double Y_LEAVE_INCHES_BLUE                = 20;
    private static final double Y_LEAVE_INCHES_RED                 = -20;
    private static final double ANGLE_LEAVE_RADIANS_RED            = -3 * Math.PI / 4;
    private static final double ANGLE_LEAVE_RADIANS_BLUE           = 3 * Math.PI / 4;



    Pose2d                  mStart          = new Pose2d(0,0,0);
    Pose2d                  mLeave          = new Pose2d(0,0,0);

    Map<Pattern, Pose2d>    mStartIntake    = new LinkedHashMap<>();
    Map<Pattern, Pose2d>    mEndIntake      = new LinkedHashMap<>();
    Map<Pattern, Pose2d>    mBackIntake     = new LinkedHashMap<>();

    double                  mTgtIntakeToShootRadians  = 0;

    public PathAutonomousGoal(Logger logger) {
        super(logger);
    }

    public void initialize(Alliance alliance) {

        super.initialize(alliance);

        if (alliance == Alliance.RED) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS_RED);

            mStartIntake.put(Pattern.GPP, new Pose2d(X_GPP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED));
            mStartIntake.put(Pattern.PGP, new Pose2d(X_PGP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED));
            mStartIntake.put(Pattern.PPG, new Pose2d(X_PPG_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED));

            for (Pattern pattern : Pattern.values()) {
                if (pattern != Pattern.NONE) {
                    mEndIntake.put(pattern, new Pose2d(
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.x,
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.y + Y_DELTA_INTAKE_INCHES_RED,
                            Objects.requireNonNull(mStartIntake.get(pattern)).heading.toDouble()));
                }
            }
            for (Pattern pattern : Pattern.values()) {
                if (pattern != Pattern.NONE) {
                    mBackIntake.put(pattern, new Pose2d(
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.x,
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.y + 0.4 * Y_DELTA_INTAKE_INCHES_RED,
                            Objects.requireNonNull(mStartIntake.get(pattern)).heading.toDouble()));
                }
            }

            mTgtIntakeToShootRadians  = TGT_INTAKE_TO_SHOOT_RADIANS_RED;
            mLeave = new Pose2d(X_LEAVE_INCHES, Y_LEAVE_INCHES_RED,ANGLE_LEAVE_RADIANS_RED);

        }

        if (alliance == Alliance.BLUE) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE, ANGLE_START_RADIANS_BLUE);

            mStartIntake.put(Pattern.GPP, new Pose2d(X_GPP_PATTERN_INCHES_BLUE,Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE));
            mStartIntake.put(Pattern.PGP, new Pose2d(X_PGP_PATTERN_INCHES_BLUE,Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE));
            mStartIntake.put(Pattern.PPG, new Pose2d(X_PPG_PATTERN_INCHES_BLUE,Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE));

            for (Pattern pattern : Pattern.values()) {

                if (pattern != Pattern.NONE) {
                    mEndIntake.put(pattern, new Pose2d(
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.x,
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                            Objects.requireNonNull(mStartIntake.get(pattern)).heading.toDouble()));
                }
            }
            for (Pattern pattern : Pattern.values()) {
                if (pattern != Pattern.NONE) {
                    mBackIntake.put(pattern, new Pose2d(
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.x,
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.y + 0.4 * Y_DELTA_INTAKE_INCHES_BLUE,
                            Objects.requireNonNull(mStartIntake.get(pattern)).heading.toDouble()));
                }
            }

            mTgtIntakeToShootRadians  = TGT_INTAKE_TO_SHOOT_RADIANS_BLUE;
            mLeave = new Pose2d(X_LEAVE_INCHES, Y_LEAVE_INCHES_BLUE,ANGLE_LEAVE_RADIANS_BLUE);

        }
    }

    public Pose2d   start()                         { return mStart; }
    public Pose2d   startIntake(Pattern pattern)    { return mStartIntake.get(pattern); }
    public Pose2d   endIntake(Pattern pattern)      { return mEndIntake.get(pattern); }
    public Pose2d   backIntake(Pattern pattern)     { return mBackIntake.get(pattern); }

    public double   tgtIntakeToShootRadians()       { return mTgtIntakeToShootRadians;}
    public Pose2d   leave()                         { return mLeave; }

    public void log() {

        mLogger.info(Logger.Target.DRIVER_STATION,"START X : " + mStart.position.x + " Y: " + mStart.position.y + " H: " + mStart.heading.toDouble());

        if(mStartIntake.get(Pattern.GPP) != null) {
            mLogger.info(Logger.Target.DRIVER_STATION,"START INTAKE GPP X : " + mStartIntake.get(Pattern.GPP).position.x + " Y: " + mStartIntake.get(Pattern.GPP).position.y + " H: " + mStartIntake.get(Pattern.GPP).heading.toDouble());
            mLogger.info(Logger.Target.DRIVER_STATION,"END INTAKE GPP X : " + mEndIntake.get(Pattern.GPP).position.x + " Y: " + mEndIntake.get(Pattern.GPP).position.y + " H: " + mEndIntake.get(Pattern.GPP).heading.toDouble());
            mLogger.info(Logger.Target.DRIVER_STATION,"BACK INTAKE GPP X : " + mBackIntake.get(Pattern.GPP).position.x + " Y: " + mBackIntake.get(Pattern.GPP).position.y + " H: " + mBackIntake.get(Pattern.GPP).heading.toDouble());
        }
        if(mStartIntake.get(Pattern.PGP) != null) {
            mLogger.info(Logger.Target.DRIVER_STATION,"START INTAKE PGP X : " + mStartIntake.get(Pattern.PGP).position.x + " Y: " + mStartIntake.get(Pattern.PGP).position.y + " H: " + mStartIntake.get(Pattern.PGP).heading.toDouble());
            mLogger.info(Logger.Target.DRIVER_STATION,"END INTAKE PGP X : " + mEndIntake.get(Pattern.PGP).position.x + " Y: " + mEndIntake.get(Pattern.PGP).position.y + " H: " + mEndIntake.get(Pattern.PGP).heading.toDouble());
            mLogger.info(Logger.Target.DRIVER_STATION,"BACK INTAKE PGP X : " + mBackIntake.get(Pattern.PGP).position.x + " Y: " + mBackIntake.get(Pattern.PGP).position.y + " H: " + mBackIntake.get(Pattern.PGP).heading.toDouble());
        }
        if(mStartIntake.get(Pattern.PPG) != null) {
            mLogger.info(Logger.Target.DRIVER_STATION,"START INTAKE PPG X : " + mStartIntake.get(Pattern.PPG).position.x + " Y: " + mStartIntake.get(Pattern.PPG).position.y + " H: " + mStartIntake.get(Pattern.PPG).heading.toDouble());
            mLogger.info(Logger.Target.DRIVER_STATION,"END INTAKE PPG X : " + mEndIntake.get(Pattern.PPG).position.x + " Y: " + mEndIntake.get(Pattern.PPG).position.y + " H: " + mEndIntake.get(Pattern.PPG).heading.toDouble());
            mLogger.info(Logger.Target.DRIVER_STATION,"BACK INTAKE PPG X : " + mBackIntake.get(Pattern.PPG).position.x + " Y: " + mBackIntake.get(Pattern.PPG).position.y + " H: " + mBackIntake.get(Pattern.PPG).heading.toDouble());
        }
        mLogger.info(Logger.Target.DRIVER_STATION,"TGT INTAKE TO SHOOT INIT : " + mTgtIntakeToShootRadians);
        mLogger.info(Logger.Target.DRIVER_STATION,"LEAVE: " + mLeave.position.x + " Y: " + mLeave.position.y + " H: " + mLeave.heading.toDouble());
        super.log();

    }

}
