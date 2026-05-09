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
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PoseConversion;
import org.firstinspires.ftc.teamcode.vision.Pattern;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

public class PathAutonomousGoalPedro extends PathPedro {

    public static final double X_START_INCHES =                     50;
    public static final double Y_START_INCHES_BLUE =                50;
    public static final double Y_START_INCHES_RED =                 - 50;
    public static final double ANGLE_START_RADIANS_RED =            - Math.PI / 180 * 50;
    public static final double ANGLE_START_RADIANS_BLUE =           Math.PI / 180 * 50;


    protected static final double Y_DELTA_INTAKE_INCHES_BLUE =      22;
    protected static final double Y_DELTA_INTAKE_INCHES_RED =       -22;

    public static final double X_GPP_PATTERN_INCHES_BLUE =          -36.25;
    public static final double X_PGP_PATTERN_INCHES_BLUE =          -15;
    public static final double X_PPG_PATTERN_INCHES_BLUE =          11.25;
    public static final double X_GPP_PATTERN_INCHES_RED =           -36.25;
    public static final double X_PGP_PATTERN_INCHES_RED =           -15;
    public static final double X_PPG_PATTERN_INCHES_RED =           11.25;


    public static final double Y_PATTERN_INCHES_BLUE =              30;
    public static final double Y_PATTERN_INCHES_RED =               -30;
    public static final double ANGLE_PATTERN_RADIANS_BLUE =         Math.PI / 2;
    public static final double ANGLE_PATTERN_RADIANS_RED =          -Math.PI / 2;

    public static final double TGT_INTAKE_TO_SHOOT_RADIANS_BLUE =   -Math.PI/4;
    public static final double TGT_INTAKE_TO_SHOOT_RADIANS_RED =    Math.PI/4;


    private static final double X_LEAVE_INCHES                     = 35;
    private static final double Y_LEAVE_INCHES_BLUE                = 15;
    private static final double Y_LEAVE_INCHES_RED                 = -15;
    private static final double ANGLE_LEAVE_RADIANS_RED            = -3 * Math.PI / 4;
    private static final double ANGLE_LEAVE_RADIANS_BLUE           = 3 * Math.PI / 4;



    Pose mStart          = new Pose(0,0,0);
    Pose                  mLeave          = new Pose(0,0,0);

    Map<Pattern, Pose>    mStartIntake    = new LinkedHashMap<>();
    Map<Pattern, Pose>    mEndIntake      = new LinkedHashMap<>();
    Map<Pattern, Pose2d>    mBackIntake     = new LinkedHashMap<>();

    double                  mTgtIntakeToShootRadians  = 0;

    public PathAutonomousGoalPedro(Logger logger) {
        super(logger);
    }

    public void initialize(Alliance alliance) {

        super.initialize(alliance);

        if (alliance == Alliance.RED) {

            mStart = PoseConversion.topedroPose(new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS_RED));

            mStartIntake.put(Pattern.GPP, PoseConversion.topedroPose(new Pose2d(X_GPP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED)));
            mStartIntake.put(Pattern.PGP, PoseConversion.topedroPose(new Pose2d(X_PGP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED)));
            mStartIntake.put(Pattern.PPG, PoseConversion.topedroPose(new Pose2d(X_PPG_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED)));

            for (Pattern pattern : Pattern.values()) {
                if (pattern != Pattern.NONE) {
                    if((pattern == Pattern.PGP) || (pattern == Pattern.GPP)){
                        mEndIntake.put(pattern, new Pose(
                            Objects.requireNonNull(mStartIntake.get(pattern)).getX(),
                            Objects.requireNonNull(mStartIntake.get(pattern)).getY() + Y_DELTA_INTAKE_INCHES_RED - 2,
                            Objects.requireNonNull(mStartIntake.get(pattern)).getHeading()));
                    }
                    else {
                        mEndIntake.put(pattern, new Pose(
                                Objects.requireNonNull(mStartIntake.get(pattern)).getX(),
                                Objects.requireNonNull(mStartIntake.get(pattern)).getY() + Y_DELTA_INTAKE_INCHES_RED,
                                Objects.requireNonNull(mStartIntake.get(pattern)).getHeading()));
                    }
                }
            }
            for (Pattern pattern : Pattern.values()) {
                if (pattern != Pattern.NONE) {
                    mBackIntake.put(pattern, new Pose2d(
                            Objects.requireNonNull(mStartIntake.get(pattern)).getX(),
                            Objects.requireNonNull(mStartIntake.get(pattern)).getY() + 0.4 * Y_DELTA_INTAKE_INCHES_RED,
                            Objects.requireNonNull(mStartIntake.get(pattern)).getHeading()));
                }
            }

            mTgtIntakeToShootRadians  = TGT_INTAKE_TO_SHOOT_RADIANS_RED;
            mLeave = PoseConversion.topedroPose(new Pose2d(X_LEAVE_INCHES, Y_LEAVE_INCHES_RED,ANGLE_LEAVE_RADIANS_RED));

        }

        if (alliance == Alliance.BLUE) {

            mStart = PoseConversion.topedroPose(new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE, ANGLE_START_RADIANS_BLUE));

            mStartIntake.put(Pattern.GPP, PoseConversion.topedroPose(new Pose2d(X_GPP_PATTERN_INCHES_BLUE,Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE)));
            mStartIntake.put(Pattern.PGP, PoseConversion.topedroPose(new Pose2d(X_PGP_PATTERN_INCHES_BLUE,Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE)));
            mStartIntake.put(Pattern.PPG, PoseConversion.topedroPose(new Pose2d(X_PPG_PATTERN_INCHES_BLUE,Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE)));

            for (Pattern pattern : Pattern.values()) {

                if (pattern != Pattern.NONE) {
                    if((pattern == Pattern.PGP) || (pattern == Pattern.GPP)){
                        mEndIntake.put(pattern, new Pose(
                            Objects.requireNonNull(mStartIntake.get(pattern)).getX(),
                            Objects.requireNonNull(mStartIntake.get(pattern)).getY() + Y_DELTA_INTAKE_INCHES_BLUE + 2,
                            Objects.requireNonNull(mStartIntake.get(pattern)).getHeading()));
                    }
                    else {
                        mEndIntake.put(pattern, new Pose(
                                Objects.requireNonNull(mStartIntake.get(pattern)).getX(),
                                Objects.requireNonNull(mStartIntake.get(pattern)).getY() + Y_DELTA_INTAKE_INCHES_BLUE,
                                Objects.requireNonNull(mStartIntake.get(pattern)).getHeading()));
                    }
                }
            }
            for (Pattern pattern : Pattern.values()) {
                if (pattern != Pattern.NONE) {
                    mBackIntake.put(pattern, new Pose2d(
                            Objects.requireNonNull(mStartIntake.get(pattern)).getX(),
                            Objects.requireNonNull(mStartIntake.get(pattern)).getY() + 0.4 * Y_DELTA_INTAKE_INCHES_BLUE,
                            Objects.requireNonNull(mStartIntake.get(pattern)).getHeading()));
                }
            }

            mTgtIntakeToShootRadians  = TGT_INTAKE_TO_SHOOT_RADIANS_BLUE;
            mLeave = PoseConversion.topedroPose(new Pose2d(X_LEAVE_INCHES, Y_LEAVE_INCHES_BLUE,ANGLE_LEAVE_RADIANS_BLUE));

        }
    }

    public Pose   start()                         { return mStart; }
    public Pose   startIntake(Pattern pattern)    { return mStartIntake.get(pattern); }
    public Pose   endIntake(Pattern pattern)      { return mEndIntake.get(pattern); }
    public Pose2d   backIntake(Pattern pattern)     { return mBackIntake.get(pattern); }

    public double   tgtIntakeToShootRadians()       { return mTgtIntakeToShootRadians;}
    public Pose   leave()                         { return mLeave; }

    public void log() {

        mLogger.info(Logger.Target.DRIVER_STATION,"START X : " + mStart.getX() + " Y: " + mStart.getY() + " H: " + mStart.getHeading());

        if(mStartIntake.get(Pattern.GPP) != null) {
            mLogger.info(Logger.Target.DRIVER_STATION,"START INTAKE GPP X : " + mStartIntake.get(Pattern.GPP).getX() + " Y: " + mStartIntake.get(Pattern.GPP).getY() + " H: " + mStartIntake.get(Pattern.GPP).getHeading());
            mLogger.info(Logger.Target.DRIVER_STATION,"END INTAKE GPP X : " + mEndIntake.get(Pattern.GPP).getX() + " Y: " + mEndIntake.get(Pattern.GPP).getY() + " H: " + mEndIntake.get(Pattern.GPP).getHeading());
            mLogger.info(Logger.Target.DRIVER_STATION,"BACK INTAKE GPP X : " + mBackIntake.get(Pattern.GPP).position.x + " Y: " + mBackIntake.get(Pattern.GPP).position.y + " H: " + mBackIntake.get(Pattern.GPP).heading.toDouble());
        }
        if(mStartIntake.get(Pattern.PGP) != null) {
            mLogger.info(Logger.Target.DRIVER_STATION,"START INTAKE PGP X : " + mStartIntake.get(Pattern.PGP).getX() + " Y: " + mStartIntake.get(Pattern.PGP).getY() + " H: " + mStartIntake.get(Pattern.PGP).getHeading());
            mLogger.info(Logger.Target.DRIVER_STATION,"END INTAKE PGP X : " + mEndIntake.get(Pattern.PGP).getX() + " Y: " + mEndIntake.get(Pattern.PGP).getY() + " H: " + mEndIntake.get(Pattern.PGP).getHeading());
            mLogger.info(Logger.Target.DRIVER_STATION,"BACK INTAKE PGP X : " + mBackIntake.get(Pattern.PGP).position.x + " Y: " + mBackIntake.get(Pattern.PGP).position.y + " H: " + mBackIntake.get(Pattern.PGP).heading.toDouble());
        }
        if(mStartIntake.get(Pattern.PPG) != null) {
            mLogger.info(Logger.Target.DRIVER_STATION,"START INTAKE PPG X : " + mStartIntake.get(Pattern.PPG).getX() + " Y: " + mStartIntake.get(Pattern.PPG).getY() + " H: " + mStartIntake.get(Pattern.PPG).getHeading());
            mLogger.info(Logger.Target.DRIVER_STATION,"END INTAKE PPG X : " + mEndIntake.get(Pattern.PPG).getX() + " Y: " + mEndIntake.get(Pattern.PPG).getY() + " H: " + mEndIntake.get(Pattern.PPG).getHeading());
            mLogger.info(Logger.Target.DRIVER_STATION,"BACK INTAKE PPG X : " + mBackIntake.get(Pattern.PPG).position.x + " Y: " + mBackIntake.get(Pattern.PPG).position.y + " H: " + mBackIntake.get(Pattern.PPG).heading.toDouble());
        }
        mLogger.info(Logger.Target.DRIVER_STATION,"TGT INTAKE TO SHOOT INIT : " + mTgtIntakeToShootRadians);
        mLogger.info(Logger.Target.DRIVER_STATION,"LEAVE: " + mLeave.getX() + " Y: " + mLeave.getY() + " H: " + mLeave.getHeading());
        super.log();

    }

}
