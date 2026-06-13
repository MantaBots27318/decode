
/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Logger
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.utils;

/* System includes */
import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.Objects;
import java.io.IOException;

/* Qualcomm includes */
import com.qualcomm.robotcore.util.ElapsedTime;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logger {

    static final int  sStackLevel = 3;

    public enum Target {
        FILE,
        DASHBOARD,
        DRIVER_STATION
    }

    public enum Severity  {
        ERROR,
        WARNING,
        METRIC,
        INFO,
        DEBUG,
        TRACE
    }

    private static final Map<String, Severity > sConfToLevel = Map.of(
            "error",    Severity.ERROR,
            "warning",  Severity.WARNING,
            "metric",   Severity.METRIC,
            "info",     Severity.INFO,
            "debug",    Severity.DEBUG,
            "trace",    Severity.TRACE
    );

    private static final Map<Severity , String > sLevelToConf = Map.of(
            Severity.ERROR,    "error",
            Severity.WARNING,  "warning",
            Severity.METRIC,   "metric",
            Severity.INFO,     "info",
            Severity.DEBUG,    "debug",
            Severity.TRACE,    "trace"
    );

    private static final Map<Severity , Integer> sLevelToPriority = Map.of(
            Severity.ERROR,    0,
            Severity.WARNING,  1,
            Severity.METRIC,   2,
            Severity.INFO,     3,
            Severity.DEBUG,    4,
            Severity.TRACE,    5
    );

    // Json keys
    static  final   String          sFilenameKey       = "file";
    static  final   String          sDriverStationKey  = "driver-station";
    static  final   String          sDashboardKey      = "panels";
    static  final   String          sLevelKey          = "level";

    // Formatting
    static  final   int             sErrorFontSize   = 14;
    static  final   int             sWarningFontSize = 14;
    public static   final   int     sMetricFontSize  = 13;
    static  final   int             sInfoFontSize    = 13;
    static  final   int             sEntryFontSize   = 15;

    // Status
    boolean                                 mConfigurationValid;
    Severity                                mLevel;
    final int                               mStackLevel;

    // Loggers
    Telemetry                               mDriverStation;
    BufferedWriter                          mFile;
    final String                            mFilename;
    final ElapsedTime                       mTimer;

    // Persistence
    final Map<Target,StringBuilder>         mErrors;
    final Map<Target,StringBuilder>         mWarnings;
    final Map<Target,Map<String,String>>    mMetrics;

    // Temporary
    final Map<Target,StringBuilder>         mInfos;
    final Map<Target,StringBuilder>         mDebugs;
    final Map<Target,StringBuilder>         mTraces;

    // Ordered by time
    StringBuilder                           mFileData;

    public Logger(Telemetry station, String filename) {
        this(station, filename, sStackLevel);
    }

    public Logger(Telemetry station, String filename, int stackLevel) {

        mConfigurationValid = true;
        mLevel = Severity.TRACE;
        mStackLevel = stackLevel;

        mErrors   = new LinkedHashMap<>();
        mWarnings = new LinkedHashMap<>();
        mInfos    = new LinkedHashMap<>();
        mDebugs   = new LinkedHashMap<>();
        mTraces   = new LinkedHashMap<>();
        mMetrics  = new LinkedHashMap<>();
        for(Target target : Target.values()) {
            mErrors.put(target,new StringBuilder());
            mWarnings.put(target,new StringBuilder());
            mInfos.put(target,new StringBuilder());
            mDebugs.put(target,new StringBuilder());
            mTraces.put(target,new StringBuilder());
            mMetrics.put(target, new LinkedHashMap<>());
        }
        mFileData = new StringBuilder();

        mTimer = new ElapsedTime();
        mTimer.reset();

        mDriverStation = station;
        if(mDriverStation != null) {
            mDriverStation.setAutoClear(true);
        }

        mFile = null;
        mFilename = filename;
        if(!mFilename.isEmpty()) {
            String filepath = System.getProperty("user.home")
                    + "/FIRST/"
                    + mFilename
                    + ".log";
            try {
                mFile = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filepath), StandardCharsets.UTF_8));
            } catch (IOException e) {
                mFile = null;
                mConfigurationValid = false;
                this.warning("Unable to open log file " + filepath);
            }
        }
    }

    public void level(Severity level) { mLevel = level; }

    public boolean isConfigured() {
        return mConfigurationValid;
    }

    public String  logConfigurationHTML() {

        String result = "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sDriverStationKey +
                ((mDriverStation == null) ? " : false" : " : true") +
                "</li>" +
                "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sDashboardKey +
                " : false" +
                "</li>" +
                "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sFilenameKey + " : " +
                ((mFile == null) ? "" : mFilename) +
                "</li>" +
                "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sLevelKey + " : " +
                sLevelToConf.get(mLevel) +
                "</li>";

        return result;
    }

    public String  logConfigurationText(String header) {

        String result = header +
                "> " +
                sDriverStationKey +
                ((mDriverStation == null) ? " : false" : " : true") +
                "\n" +
                header +
                "> " +
                sDashboardKey +
                " : false" +
                "\n" +
                header +
                "> " +
                sFilenameKey + " : " +
                ((mFile == null) ? "" : mFilename) +
                "\n" +
                header +
                "> " +
                sLevelKey + " : " +
                sLevelToConf.get(mLevel) +
                "\n";

        return result;
    }

    public void error(Target target, String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        this.error(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
    }

    public void error(String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        for(Target target : Target.values()) {
            this.error(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
        }
    }

    public void warning(Target target, String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        this.warning(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
    }

    public void warning(String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        for(Target target : Target.values()) {
            this.warning(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
        }
    }

    public void metric(String metric, String value) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        for(Target target : Target.values()) {
            this.metric(target, metric, value, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
        }
    }

    public void metric(Target target, String metric, String value) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        this.metric(target, metric, value, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
    }

    public void info(String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        for(Target target : Target.values()) {
            this.info(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
        }
    }

    public void info(Target target, String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        this.info(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
    }

    public void debug(String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        for(Target target : Target.values()) {
            this.debug(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
        }
    }

    public void debug(Target target, String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        this.debug(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
    }

    public void trace(String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        for(Target target : Target.values()) {
            this.trace(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
        }
    }

    public void trace(Target target, String message) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[mStackLevel];
        this.trace(target, message, element.getFileName().substring(0, element.getFileName().lastIndexOf(".")), element.getMethodName(), element.getLineNumber());
    }

    public void write(Target target) {
        if (target == Target.DRIVER_STATION && mDriverStation != null) {
            mDriverStation.addLine("---------- ERRORS ----------");
            mDriverStation.addLine(Objects.requireNonNull(mErrors.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("--------- WARNINGS ---------");
            mDriverStation.addLine(Objects.requireNonNull(mWarnings.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("---------- METRICS ---------");
            for (Map.Entry<String, String> metric : Objects.requireNonNull(mMetrics.get(target)).entrySet()) {
                mDriverStation.addLine(metric.getKey() + " : " + metric.getValue());
            }
            mDriverStation.addLine("");
            mDriverStation.addLine("---------- INFOS ----------");
            mDriverStation.addLine(Objects.requireNonNull(mInfos.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("---------- DEBUGS ---------");
            mDriverStation.addLine(Objects.requireNonNull(mDebugs.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("---------- TRACES ----------");
            mDriverStation.addLine(Objects.requireNonNull(mTraces.get(Target.DRIVER_STATION)).toString());
        } else if(target == Target.FILE && mFile != null) {
            try {
                mFile.write(mFileData.toString());
                mFileData = new StringBuilder();
            }
            catch(IOException e) { this.warning(e.getMessage()); }
        }
    }

    public void raw(Target target, String raw) {

        Integer infoPriority = sLevelToPriority.get(Severity.INFO);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && infoPriority != null && filterPriority >= infoPriority) {
            if (target == Target.DRIVER_STATION && mDriverStation != null) {
                Objects.requireNonNull(mInfos.get(target)).append(raw);
            } else if (target == Target.FILE && mFile != null) {
                mFileData.append(raw);
            }
        }
    }

    public void update(Target target) {
        if (target == Target.DRIVER_STATION && mDriverStation != null) {
            this.write(target);
            mDriverStation.update();
        } else if (target == Target.FILE && mFile != null) {
            this.write(target);
        }
        mInfos.put(target,new StringBuilder());
        mDebugs.put(target,new StringBuilder());
        mTraces.put(target,new StringBuilder());
    }

    public void update() {
        for(Target target : Target.values()) {
            this.update(target);
        }
    }

    public void clear(Target target) {
        if (target == Target.DRIVER_STATION && mDriverStation != null) {
            mDriverStation.clear();
        }
    }

    public void clear() {
        for(Target target : Target.values()) {
            this.clear(target);
        }
    }

    public void reset(Target target) {
        mErrors.put(target,new StringBuilder());
        mWarnings.put(target,new StringBuilder());
        mInfos.put(target,new StringBuilder());
        mDebugs.put(target,new StringBuilder());
        mTraces.put(target,new StringBuilder());
        mMetrics.put(target, new LinkedHashMap<>());
        mFileData = new StringBuilder();
    }

    public void reset() {
        for(Target target : Target.values()) {
            this.reset(target);
        }
    }

    public void stop() {
        for(Target target : Target.values()) {
            this.write(target);
        }
        if(mFile != null) {
            try {
                mFile.flush();
                mFile.close();
            }
            catch(IOException e) { this.warning(e.getMessage()); }
        }
    }

    private void error(Target target, String message, String className, String methodName, int line) {

        Integer errorPriority = sLevelToPriority.get(Severity.ERROR);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && errorPriority != null && filterPriority >= errorPriority) {

            final String number = line < 1000 ? (line < 100 ? (line < 10 ? "000" : "00") : "0") : "";
            switch (target) {
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mErrors.get(target))
                                .append(className).append(".").append(methodName).append(":")
                                .append(number).append(line).append(" - ").append(message).append("\n");
                    }
                    break;
                case FILE:
                    double elapsedTime = mTimer.seconds();
                    int minutes = (int)(elapsedTime / 60);
                    int hours = (int)(elapsedTime / 3600);
                    double seconds = (double)((int)((elapsedTime - 60 * minutes - 3600 * hours) * 1000)) / 1000.0;
                    mFileData.append("[").append(minutes).append(':').append(seconds)
                            .append("] [ERROR] - ").append(className).append('.').append(methodName).append(':')
                            .append(number).append(line).append(" - ").append(message).append('\n');
                    break;
                default:
                    break;
            }
        }
    }

    private void warning(Target target, String message, String className, String methodName, int line) {
        Integer warningPriority = sLevelToPriority.get(Severity.WARNING);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && warningPriority != null && filterPriority >= warningPriority) {

            switch (target) {
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mWarnings.get(target))
                                .append(className).append(".").append(methodName).append(":")
                                .append(line < 1000 ? (line < 100 ? (line < 10 ? "000" : "00") : "0") : "").append(line)
                                .append(" - ").append(message).append("\n");
                    }
                    break;
                case FILE:
                    double elapsedTime = mTimer.seconds();
                    int minutes = (int)(elapsedTime / 60);
                    int hours = (int)(elapsedTime / 3600);
                    double seconds = (double)((int)((elapsedTime - 60 * minutes - 3600 * hours) * 1000)) / 1000.0;
                    mFileData.append("[").append(minutes).append(':').append(seconds)
                            .append("] [WARNING] - ").append(className).append('.').append(methodName).append(':')
                            .append(line < 1000 ? (line < 100 ? (line < 10 ? "000" : "00") : "0") : "").append(line)
                            .append(" - ").append(message).append('\n');
                    break;
                default:
                    break;
            }
        }
    }

    private void metric(Target target, String metric, String value, String className, String methodName, int line) {

        Integer metricPriority = sLevelToPriority.get(Severity.METRIC);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && metricPriority != null && filterPriority >= metricPriority) {

            switch (target) {
                case DRIVER_STATION:
                case DASHBOARD:
                    Objects.requireNonNull(mMetrics.get(target)).put(metric, value);
                    break;
                case FILE:
                    double elapsedTime = mTimer.seconds();
                    int minutes = (int)(elapsedTime / 60);
                    int hours = (int)(elapsedTime / 3600);
                    double seconds = (double)((int)((elapsedTime - 60 * minutes - 3600 * hours) * 1000)) / 1000.0;
                    mFileData.append("[").append(minutes).append(':').append(seconds)
                            .append("] [METRIC] - ").append(className).append('.').append(methodName).append(':')
                            .append(line < 1000 ? (line < 100 ? (line < 10 ? "000" : "00") : "0") : "").append(line)
                            .append(" - ").append(metric).append(" : ").append(value).append('\n');
                    break;
            }
        }
    }

    private void info(Target target, String message, String className, String methodName, int line) {

        Integer infoPriority = sLevelToPriority.get(Severity.INFO);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && infoPriority != null && filterPriority >= infoPriority) {
            switch (target) {
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mInfos.get(target)).append(message).append("\n");
                    }
                    break;
                case FILE:
                    double elapsedTime = mTimer.seconds();
                    int minutes = (int)(elapsedTime / 60);
                    int hours = (int)(elapsedTime / 3600);
                    double seconds = (double)((int)((elapsedTime - 60 * minutes - 3600 * hours) * 1000)) / 1000.0;
                    mFileData.append("[").append(minutes).append(':').append(seconds)
                            .append("] [INFO] - ").append(className).append('.').append(methodName).append(':')
                            .append(line < 1000 ? (line < 100 ? (line < 10 ? "000" : "00") : "0") : "").append(line)
                            .append(" - ").append(message).append('\n');
                    break;
                default:
                    break;
            }
        }
    }

    private void debug(Target target, String message, String className, String methodName, int line) {

        Integer debugPriority = sLevelToPriority.get(Severity.DEBUG);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && debugPriority != null && filterPriority >= debugPriority) {

            switch (target) {
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mDebugs.get(target)).append(message).append("\n");
                    }
                    break;
                case FILE:
                    double elapsedTime = mTimer.seconds();
                    int minutes = (int)(elapsedTime / 60);
                    int hours = (int)(elapsedTime / 3600);
                    double seconds = (double)((int)((elapsedTime - 60 * minutes - 3600 * hours) * 1000)) / 1000.0;
                    mFileData.append("[").append(minutes).append(':').append(seconds)
                            .append("] [DEBUG] - ").append(className).append('.').append(methodName).append(':')
                            .append(line < 1000 ? (line < 100 ? (line < 10 ? "000" : "00") : "0") : "").append(line)
                            .append(" - ").append(message).append('\n');
                    break;
                default:
                    break;
            }
        }
    }

    private void trace(Target target, String message, String className, String methodName, int line) {

        Integer tracePriority = sLevelToPriority.get(Severity.TRACE);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && tracePriority != null && filterPriority >= tracePriority) {

            switch (target) {
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mTraces.get(target)).append(message).append("\n");
                    }
                    break;
                case FILE:
                    double elapsedTime = mTimer.seconds();
                    int minutes = (int)(elapsedTime / 60);
                    int hours = (int)(elapsedTime / 3600);
                    double seconds = (double)((int)((elapsedTime - 60 * minutes - 3600 * hours) * 1000)) / 1000.0;
                    mFileData.append("[").append(minutes).append(':').append(seconds)
                            .append("] [TRACE] - ").append(className).append('.').append(methodName).append(':')
                            .append(line < 1000 ? (line < 100 ? (line < 10 ? "000" : "00") : "0") : "").append(line)
                            .append(" - ").append(message).append('\n');
                    break;
                default:
                    break;
            }
        }
    }
}