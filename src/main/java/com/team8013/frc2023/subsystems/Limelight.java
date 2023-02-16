package com.team8013.frc2023.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.RobotState;
import com.team8013.frc2023.controlboard.ControlBoard;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team8013.frc2023.loops.ILooper;
import com.team8013.frc2023.loops.Loop;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight extends Subsystem {
    private static Limelight mInstance = null;
    private LimelightConstants mConstants = null;
    private NetworkTable mNetworkTable;
    LogStorage<PeriodicIO> mStorage = null;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private int mLatencyCounter = 0;
    public Optional<Double> mDistanceToTarget = Optional.empty();
    private boolean mOutputsHaveChanged = true;

    double KpAim = 1; // controls overshoot of aim
    double minCommand = 0.275; // controls minimum voltage of aim
    double headingError = -mPeriodicIO.tx;
    double steeringAdjust = 0.0;
    double drivingAdjust = 0.0;
    double KpDistance = -0.1;

    public final static int kDefaultPipeline = 0;
    public final static int kZoomedInPipeline = 1;


    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "limelight";
        public double kHeight = 0.0; // height of limelight from ground?
        public Rotation2d kHorizontalPlaneToLens = Rotation2d.identity();
    }

    private Limelight() {
        mConstants = Constants.VisionConstants.kLimelightConstants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName);
    }

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    /** Returns steering adjustment calculated from the horizontal crosshair offset.
     * @return A double
     */
    public double getSteeringAdjust() {
        if (mPeriodicIO.tx > 1.0) {
            steeringAdjust = KpAim * headingError - minCommand;
        }
        else if (mPeriodicIO.tx < -1.0) {
            steeringAdjust = KpAim * headingError + minCommand;
        }

        return steeringAdjust;
    }
    
    /** Returns driving adjustment calculated from the vertical crosshair offset.
     * @return A double
     */
    public double getDrivingAdjust() {
        return KpDistance * mPeriodicIO.ty;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                RobotState.getInstance().resetVision();
                setLed(LedMode.ON);
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();

                synchronized (Limelight.this) {
                    List<TargetInfo> targetInfo = getTarget();
                    if (mPeriodicIO.sees_target && targetInfo != null) {
                        RobotState.getInstance().addVisionUpdate(timestamp - getLatency(), getTarget(), Limelight.this);
                        updateDistanceToTarget();
                    }
                }

                // send log data
                SendLog();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                setLed(LedMode.OFF);
            }
        };
        mEnabledLooper.register(mLoop);
    }
    public static class PeriodicIO {
        // INPUTS
        public int givenLedMode;
        public int givenPipeline;
        public double tx; // Horizontal offset from crosshair to target 
        public double ty; // Vertical offset from crosshair to target
        public double dt;
        public double latency;
        public double area;
        public boolean has_comms;
        public boolean sees_target;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = new ArrayList<TargetInfo>(); // getRawTargetInfos();

        targets.add(new TargetInfo(Math.tan(Math.toRadians(-mPeriodicIO.tx)),
                Math.tan(Math.toRadians(mPeriodicIO.ty))));

        if (hasTarget() && targets != null) {
            return targets;
        }
        else {
            return null;
        }
    }

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return mConstants.kHorizontalPlaneToLens;
    }

    public Optional<Double> getLimelightDistanceToTarget() {
        return mDistanceToTarget;
    }

    public void updateDistanceToTarget() {
        double goal_theta = Constants.VisionConstants.kLimelightConstants.kHorizontalPlaneToLens.getRadians()
                + Math.toRadians(mPeriodicIO.ty);
        double height_diff = Constants.VisionConstants.kGoalHeight
                - Constants.VisionConstants.kLimelightConstants.kHeight;

        mDistanceToTarget = Optional.of(height_diff / Math.tan(goal_theta) + Constants.VisionConstants.kGoalRadius); // add
                                                                                                                     // goal
                                                                                                                     // radius
                                                                                                                     // for
                                                                                                                     // offset
                                                                                                                     // to
                                                                                                                     // center
                                                                                                                     // of
                                                                                                                     // target
    }

    @Override
    public synchronized void readPeriodicInputs() {
        final double latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0
                + Constants.VisionConstants.kImageCaptureLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.tx = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.ty = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);

        if (latency == mPeriodicIO.latency) {
            mLatencyCounter++;
        } else {
            mLatencyCounter = 0;
        }

        mPeriodicIO.latency = latency;
        mPeriodicIO.has_comms = mLatencyCounter < 10;

        mPeriodicIO.sees_target = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode || mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {

            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("Limelight Ok", mPeriodicIO.has_comms);
        SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", mPeriodicIO.latency);
        SmartDashboard.putNumber("Limelight dt", mPeriodicIO.dt);

        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mPeriodicIO.sees_target);
        SmartDashboard.putNumber("Limelight Tx: ", mPeriodicIO.tx);
        SmartDashboard.putNumber("Limelight Ty: ", mPeriodicIO.ty);

        SmartDashboard.putNumber("Limelight Distance To Target",
                mDistanceToTarget.isPresent() ? mDistanceToTarget.get() : 0.0);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized boolean isAimed() {
        if (hasTarget()) {
            return Util.epsilonEquals(mPeriodicIO.tx, 0.0, Constants.VisionAlignConstants.kEpsilon);
        } else {
            return false;
        }
    }

    public synchronized boolean isAutonomousAimed() {
        if (hasTarget()) {
            return Util.epsilonEquals(mPeriodicIO.tx, 0.0, 1.0);
        } else {
            return false;
        }
    }

    

    public synchronized boolean limelightOK() {
        return mPeriodicIO.has_comms;
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    public double getDt() {
        return mPeriodicIO.dt;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized boolean hasTarget() {
        return mPeriodicIO.sees_target;
    }

    public synchronized boolean isOK() {
        return mPeriodicIO.has_comms;
    }

    /** Returns horizontal and verical crosshair offsets
     * @return A double array (tx, ty)
     */
    public double[] getOffset() {
        return new double[] { mPeriodicIO.tx, mPeriodicIO.ty };
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "LIMELIGHT_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");

        headers.add("has_comms");
        headers.add("dt");
        headers.add("latency");

        headers.add("xOffset");
        headers.add("yOffset");
        headers.add("area");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());

        items.add(mPeriodicIO.has_comms ? 1.0 : 0.0);
        items.add(mPeriodicIO.dt);
        items.add(mPeriodicIO.latency);

        items.add(mPeriodicIO.tx);
        items.add(mPeriodicIO.ty);
        items.add(mPeriodicIO.area);

        // send data to logging storage
        mStorage.addData(items);
    }
}
