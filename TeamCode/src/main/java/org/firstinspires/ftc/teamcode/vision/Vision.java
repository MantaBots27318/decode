/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Class managing limelight pipelines and results
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.vision;

// Java includes
import java.util.ArrayList;
import java.util.List;

// Qualcomm includes
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

// FTC Controller includes
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;

// Opencv includes
import org.opencv.core.Point;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Vision {

    boolean              mReady;       // True if component is able to fulfil its mission

    private final   Limelight3A  mLimelight;
    private final   Logger       mLogger;
    private         Calibration  mCalibration;

    private         Integer      mAprilTagPipeline;
    private         Integer      mDetectionPipeline;
    private         Integer      mCurrentPipeline;

    public Vision(ConfLimelight conf, HardwareMap hwMap, String name, Logger logger) {

        mLogger       = logger;

        mReady        = true;
        String status = "";
        mCurrentPipeline = -1;

        String camera_name = conf.getHw();
        mLimelight = hwMap.tryGet(Limelight3A.class, camera_name);
        if(mLimelight == null)  { mReady = false; status += " HW";}
        else {
            mAprilTagPipeline = conf.getPipeline("localizer");
            mDetectionPipeline = conf.getPipeline("balls-detector");

            if (mAprilTagPipeline == -1)  { mReady = false; status += " PPL A"; }
            if (mDetectionPipeline == -1) { status += " PPL D"; }

            mCalibration = new Calibration();
        }

        // Log status
        if (mReady) { logger.info("==>  VISION : OK"); }
        else        { logger.warning("==>  VISION : KO : " + status); }


    }

    public void initialize() {

        if(mReady) {
            try {
                mLimelight.start();
                mCalibration.warp();
            } catch (Exception e) { mLogger.error(e.getMessage()); }
        }

    }

    public void close() {
        if(mReady) {
            try                 { mLimelight.stop(); }
            catch (Exception e) { mLogger.error(e.getMessage()); }
        }
    }

    public Pattern readPattern() {

        Pattern result = Pattern.NONE;

        if(mReady) {

            int apriltagId = 0;

            if(mCurrentPipeline != mAprilTagPipeline) {
                mLimelight.pipelineSwitch(mAprilTagPipeline);
                mCurrentPipeline = mAprilTagPipeline;
            }
            LLResult res = mLimelight.getLatestResult();

            if (res != null) {

                mLogger.trace("Limelight result valid " + res.isValid());
                if (res.isValid()) {

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = res.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        if (fr.getFiducialId() > 20 && fr.getFiducialId() < 24) {
                            apriltagId = fr.getFiducialId();
                        }

                    }
                    if (apriltagId == 22) { result = Pattern.PGP; }
                    if (apriltagId == 23) { result = Pattern.PPG; }
                    if (apriltagId == 21) { result = Pattern.GPP; }
                }
            }
        }

        return result;
    }

    public List<Ball> getArtifactPosition(){

        List<Ball> result = new ArrayList<>();

        if(mReady) {

            if(mCurrentPipeline != mDetectionPipeline) {
                mLimelight.pipelineSwitch(mDetectionPipeline);
                mCurrentPipeline = mDetectionPipeline;
            }
            LLResult llresult = mLimelight.getLatestResult();

            if (llresult != null) {

                mLogger.trace("Limelight result valid " + llresult.isValid());
                if (llresult.isValid()) {
                    List<LLResultTypes.DetectorResult> detectorResults = llresult.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {

                        double pixelX = dr.getTargetXPixels();
                        double pixelY = dr.getTargetYPixels();

                        Point inputPixelPoint = new Point(pixelX, pixelY);
                        Point detectedMatPoint = mCalibration.distance(inputPixelPoint);
                        String detectedColor = dr.getClassName();
                        mLogger.debug(String.format("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea()));
                        Ball detected = new Ball(detectedColor, detectedMatPoint);

                        result.add(detected);
                    }
                }
            }
        }

        return result ;
    }

    public Pose3D getPosition(){

        Pose3D result = null;

        if(mReady) {

            mLogger.trace(Logger.Target.FILE, "before pipeline switch");
            if(mCurrentPipeline != mAprilTagPipeline) {
                mLimelight.pipelineSwitch(mAprilTagPipeline);
                mCurrentPipeline = mAprilTagPipeline;
            }
            mLogger.trace(Logger.Target.FILE, "after pipeline switch");
            LLResult llresult = mLimelight.getLatestResult();
            if (llresult != null) {
                if (llresult.isValid()) {
                    result = llresult.getBotpose();
                }
            }
            mLogger.trace(Logger.Target.FILE, "after result processing");

        }
        return result;

    }

    public Pose3D getRelativePosition(){

        Pose3D result = null;

        if(mReady) {

            if(mCurrentPipeline != mAprilTagPipeline) {
                mLimelight.pipelineSwitch(mAprilTagPipeline);
                mCurrentPipeline = mAprilTagPipeline;
            }
            LLResult llresult = mLimelight.getLatestResult();
            if (llresult != null) {
                if (llresult.isValid()) {
                    List<LLResultTypes.FiducialResult> qrcodes = llresult.getFiducialResults();
                    if (!qrcodes.isEmpty()) {
                        result = qrcodes.get(0).getCameraPoseTargetSpace();
                    }
                }
            }

        }
        return result;

    }


}
