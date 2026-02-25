/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Class managing path reference positions for autonomous
   middle
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.pose;

/* ACME includes */
import com.acmerobotics.roadrunner.Pose2d;

/* Project includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Pattern;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

public class PathAutonomousMiddle extends Path {

    private static final double X_START_INCHES =                            -53.5 - 5;
    private static final double Y_START_INCHES_BLUE =                       18;
    private static final double Y_START_INCHES_RED =                        -18;
    private static final double ANGLE_START_RADIANS =                       0;


    protected static final double Y_DELTA_INTAKE_INCHES_BLUE =              41;
    protected static final double Y_DELTA_INTAKE_INCHES_RED =               -41;


    private static final double X_DELTA_GPP_PATTERN_INCHES_BLUE =           19;
    private static final double X_DELTA_PGP_PATTERN_INCHES_BLUE =           40;
    private static final double X_DELTA_PPG_PATTERN_INCHES_BLUE =           68;
    private static final double X_DELTA_GPP_PATTERN_INCHES_RED =            24;
    private static final double X_DELTA_PGP_PATTERN_INCHES_RED =            45;
    private static final double X_DELTA_PPG_PATTERN_INCHES_RED =            71;
    private static final double Y_DELTA_PATTERN_INCHES_BLUE =               10;
    private static final double Y_DELTA_PATTERN_INCHES_RED =                -10;
    private static final double ANGLE_DELTA_PATTERN_RADIANS_BLUE =          Math.PI / 2;
    private static final double ANGLE_DELTA_PATTERN_RADIANS_RED =           -Math.PI / 2;

    private static final double TGT_SHOOT_TO_LEAVE_RADIANS_BLUE =           Math.PI/2;
    private static final double TGT_SHOOT_TO_LEAVE_RADIANS_RED =            -Math.PI/2;

    private static final double X_LEAVE_VERY_FAR_INCHES = -52;
    private static final double Y_LEAVE_VERY_FAR_INCHES_BLUE = 19;
    private static final double ANGLE_LEAVE_VERY_FAR_RADIANS_BLUE = 0.365424564;
    private static final double Y_LEAVE_VERY_FAR_INCHES_RED = -19;
    private static final double ANGLE_LEAVE_VERY_FAR_RADIANS_RED = -0.365424564;


    Pose2d  mStart                          = new Pose2d(0,0,0);
    Map<Pattern, Pose2d>    mStartIntake    = new LinkedHashMap<>();
    Map<Pattern, Pose2d>    mEndIntake      = new LinkedHashMap<>();
    Map<Pattern, Pose2d>    mBackIntake     = new LinkedHashMap<>();
    Pose2d  mLeaveVeryFar                   = new Pose2d(0,0,0);
    double  mTgtShootToLeaveRadians         = 0;

    public PathAutonomousMiddle(Logger logger) {
        super(logger);
    }

    public void initialize(Alliance alliance) {

        super.initialize(alliance);

        if (alliance == Alliance.RED) {

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_RED;

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS);

            mStartIntake.put(Pattern.GPP, new Pose2d(
                        X_START_INCHES + X_DELTA_GPP_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED));

            mStartIntake.put(Pattern.PGP, new Pose2d(
                    X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_RED,
                    Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                    ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED));

            mStartIntake.put(Pattern.PPG, new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED));


            for (Pattern pattern : Pattern.values()) {

                mEndIntake.put(pattern, new Pose2d(
                        Objects.requireNonNull(mStartIntake.get(pattern)).position.x,
                        Objects.requireNonNull(mStartIntake.get(pattern)).position.y + Y_DELTA_INTAKE_INCHES_RED,
                        Objects.requireNonNull(mStartIntake.get(pattern)).heading.toDouble()));
            }
            for (Pattern pattern : Pattern.values()) {

                mBackIntake.put(pattern, new Pose2d(
                        Objects.requireNonNull(mStartIntake.get(pattern)).position.x,
                        Objects.requireNonNull(mStartIntake.get(pattern)).position.y + 0.3 * Y_DELTA_INTAKE_INCHES_RED,
                        Objects.requireNonNull(mStartIntake.get(pattern)).heading.toDouble()));
            }

            mTgtShootToLeaveRadians = TGT_SHOOT_TO_LEAVE_RADIANS_RED + ANGLE_START_RADIANS;
            mLeaveVeryFar = new Pose2d(X_LEAVE_VERY_FAR_INCHES,Y_LEAVE_VERY_FAR_INCHES_RED,ANGLE_LEAVE_VERY_FAR_RADIANS_RED);

        }
        if (alliance == Alliance.BLUE) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE,ANGLE_START_RADIANS);

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_BLUE;

            mStartIntake.put(Pattern.GPP, new Pose2d(
                        X_START_INCHES + X_DELTA_GPP_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE));
            mStartIntake.put(Pattern.PGP, new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE));
            mStartIntake.put(Pattern.PPG, new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE));

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
                            Objects.requireNonNull(mStartIntake.get(pattern)).position.y + 0.3 * Y_DELTA_INTAKE_INCHES_BLUE,
                            Objects.requireNonNull(mStartIntake.get(pattern)).heading.toDouble()));
                }
            }

            mTgtShootToLeaveRadians = TGT_SHOOT_TO_LEAVE_RADIANS_BLUE + ANGLE_START_RADIANS;
            mLeaveVeryFar = new Pose2d(X_LEAVE_VERY_FAR_INCHES,Y_LEAVE_VERY_FAR_INCHES_BLUE,ANGLE_LEAVE_VERY_FAR_RADIANS_BLUE);

        }
    }

    public Pose2d   start()                         { return mStart; }
    public Pose2d   startIntake(Pattern pattern)    { return mStartIntake.get(pattern); }
    public Pose2d   endIntake(Pattern pattern)      { return mEndIntake.get(pattern); }
    public Pose2d   backIntake(Pattern pattern)     { return mBackIntake.get(pattern); }
    public Pose2d   leaveVeryFar()                  { return mLeaveVeryFar; }

    public double   tgtShootToLeaveRadians()       { return mTgtShootToLeaveRadians;}

    public void log() {

        mLogger.info("START X : " + mStart.position.x + " Y: " + mStart.position.y + " H: " + mStart.heading.toDouble());
        if(mStartIntake.get(Pattern.GPP) != null) {
            mLogger.info("START INTAKE GPP X : " + mStartIntake.get(Pattern.GPP).position.x + " Y: " + mStartIntake.get(Pattern.GPP).position.y + " H: " + mStartIntake.get(Pattern.GPP).heading.toDouble());
            mLogger.info("END INTAKE GPP X : " + mEndIntake.get(Pattern.GPP).position.x + " Y: " + mEndIntake.get(Pattern.GPP).position.y + " H: " + mEndIntake.get(Pattern.GPP).heading.toDouble());
            mLogger.info("BACK INTAKE GPP X : " + mBackIntake.get(Pattern.GPP).position.x + " Y: " + mBackIntake.get(Pattern.GPP).position.y + " H: " + mBackIntake.get(Pattern.GPP).heading.toDouble());
        }
        if(mStartIntake.get(Pattern.PGP) != null) {
            mLogger.info("START INTAKE PGP X : " + mStartIntake.get(Pattern.PGP).position.x + " Y: " + mStartIntake.get(Pattern.PGP).position.y + " H: " + mStartIntake.get(Pattern.PGP).heading.toDouble());
            mLogger.info("END INTAKE PGP X : " + mEndIntake.get(Pattern.PGP).position.x + " Y: " + mEndIntake.get(Pattern.PGP).position.y + " H: " + mEndIntake.get(Pattern.PGP).heading.toDouble());
            mLogger.info("BACK INTAKE PGP X : " + mBackIntake.get(Pattern.PGP).position.x + " Y: " + mBackIntake.get(Pattern.PGP).position.y + " H: " + mBackIntake.get(Pattern.PGP).heading.toDouble());
        }
        if(mStartIntake.get(Pattern.PPG) != null) {
            mLogger.info("START INTAKE PPG X : " + mStartIntake.get(Pattern.PPG).position.x + " Y: " + mStartIntake.get(Pattern.PPG).position.y + " H: " + mStartIntake.get(Pattern.PPG).heading.toDouble());
            mLogger.info("END INTAKE PPG X : " + mEndIntake.get(Pattern.PPG).position.x + " Y: " + mEndIntake.get(Pattern.PPG).position.y + " H: " + mEndIntake.get(Pattern.PPG).heading.toDouble());
            mLogger.info("BACK INTAKE PPG X : " + mBackIntake.get(Pattern.PPG).position.x + " Y: " + mBackIntake.get(Pattern.PPG).position.y + " H: " + mBackIntake.get(Pattern.PPG).heading.toDouble());
        }
        mLogger.info("LEAVE : " + mLeaveVeryFar.position.x + " Y: " + mLeaveVeryFar.position.y + " H: " + mLeaveVeryFar.heading.toDouble());
        mLogger.info("TGT SHOOT TO LEAVE : " + mTgtShootToLeaveRadians);
        super.log();
    }

}
