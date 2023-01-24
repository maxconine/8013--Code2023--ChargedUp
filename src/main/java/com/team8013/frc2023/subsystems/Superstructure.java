package com.team8013.frc2023.subsystems;

import com.team254.lib.vision.AimingParameters;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.RobotState;
import com.team8013.frc2023.controlboard.ControlBoard;
import com.team8013.frc2023.drivers.Pigeon;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team8013.frc2023.loops.ILooper;
import com.team8013.frc2023.loops.Loop;
import com.team254.lib.geometry.Pose2d;


import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Optional;

public class Superstructure extends Subsystem {

    // superstructure instance
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    /*** REQUIRED INSTANCES ***/
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    //private final Intake mIntake = Intake.getInstance();
    //private final Indexer mIndexer = Indexer.getInstance();
    //private final ColorSensor mColorSensor = ColorSensor.getInstance();
    //private final Shooter mShooter = Shooter.getInstance();
    //private final Trigger mTrigger = Trigger.getInstance();
    //private final Hood mHood = Hood.getInstance();
    //private final Climber mClimber = Climber.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    //private final LEDs mLEDs = LEDs.getInstance();
    private final Pigeon mPigeon = Pigeon.getInstance();

    // robot state
    private final RobotState mRobotState = RobotState.getInstance();

    // timer for reversing the intake and then stopping it once we have two correct
    // cargo
    Timer mIntakeRejectTimer = new Timer();
    // timer for asserting ball position
    Timer mAssertBallPositionTimer = new Timer();

    // PeriodicIO instance and paired csv writer
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    /*** CONTAINER FOR SUPERSTRUCTURE ACTIONS AND GOALS ***/
    public static class PeriodicIO {
        // INPUTS
        // (superstructure actions)
        // private boolean INTAKE = false; // run the intake to pick up cargo
        // private boolean REVERSE = false; // reverse the intake and singulator
        // private boolean REJECT = false; // have the intake reject cargo

        // time measurements
        public double timestamp;
        public double dt;
    }

    //     // OUTPUTS
    double mRoll = 0;

    // // aiming parameter vars
    private Optional<AimingParameters> real_aiming_params_ = Optional.empty();
    private int mTrackId = -1;
    private double mTargetAngle = 0.0;
    public double mCorrectedDistanceToTarget = 0.0;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();


                //updateVisionAimingParameters();

                // setGoals();
                // updateRumble();

                // send log data
                SendLog();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }


    /*** RUMBLE OPERATOR CONTROLLERS ***/
    public void updateRumble() {
            mControlBoard.setOperatorRumble(false);
    }
    

    /***
     * GET REAL AIMING PARAMETERS
     * called in updateVisionAimingSetpoints()
     */
    // public Optional<AimingParameters> getRealAimingParameters() {
    //     Optional<AimingParameters> aiming_params = RobotState.getInstance().getAimingParameters(mTrackId,
    //             Constants.VisionConstants.kMaxGoalTrackAge);
    //     if (aiming_params.isPresent()) {
    //         return aiming_params;
    //     } else {
    //         Optional<AimingParameters> default_aiming_params = RobotState.getInstance().getDefaultAimingParameters();
    //         return default_aiming_params;
    //     }
    // }

    // /*** UPDATE VISION AIMING PARAMETERS FROM GOAL TRACKING ***/
    // public void updateVisionAimingParameters() {
    //     // get aiming parameters from either vision-assisted goal tracking or
    //     // odometry-only tracking
    //     real_aiming_params_ = getRealAimingParameters();

    //     // predicted pose and target
    //     Pose2d predicted_field_to_vehicle = mRobotState
    //             .getPredictedFieldToVehicle(Constants.VisionConstants.kLookaheadTime);
    //     Pose2d predicted_vehicle_to_goal = predicted_field_to_vehicle.inverse()
    //             .transformBy(real_aiming_params_.get().getFieldToGoal());

    //     // update align delta from target and distance from target
    //     mTrackId = real_aiming_params_.get().getTrackId();
    //     mTargetAngle = predicted_vehicle_to_goal.getTranslation().direction().getRadians() + Math.PI;

    //     // send vision aligning target delta to swerve
    //     mSwerve.acceptLatestGoalTrackVisionAlignGoal(mTargetAngle);

    //     // update distance to target
    //     if (mLimelight.hasTarget() && mLimelight.getLimelightDistanceToTarget().isPresent()) {
    //         mCorrectedDistanceToTarget = mLimelight.getLimelightDistanceToTarget().get();
    //     } else {
    //         mCorrectedDistanceToTarget = predicted_vehicle_to_goal.getTranslation().norm();
    //     }
    // }


    // /*** UPDATE STATUS LEDS ON ROBOT ***/
    // public void updateLEDs() {
    //     if (mLEDs.getUsingSmartdash()) {
    //         return;
    //     }

    //     State topState = State.OFF;
    //     State bottomState = State.OFF;

    //     if (hasEmergency) {
    //         topState = State.EMERGENCY;
    //         bottomState = State.EMERGENCY;
            // else{
            //         State topState = State.OFF;
    //         State bottomState = State.OFF;
    //      }

    //     mLEDs.applyStates(topState, bottomState);
    // }

    // // get vision align delta from goal
    // public double getVisionAlignGoal() {
    //     return mTargetAngle;
    // }

    // check if our limelight sees a vision target
    public boolean hasTarget() {
        return mLimelight.hasTarget();
    }

    // // checked if we are vision aligned to the target within an acceptable horiz. error
    // public boolean isAimed() {
    //     return mLimelight.isAimed();
    // }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void stop() {
        // mPeriodicIO.INTAKE = false;
        // mPeriodicIO.REVERSE = false;
        // mPeriodicIO.REJECT = false;
    }

    /* Initial states for superstructure for teleop */
    public void setInitialTeleopStates() {
        // mPeriodicIO.INTAKE = false;
        // mPeriodicIO.REVERSE = false;
        // mPeriodicIO.REJECT = false;
        // mPeriodicIO.PREP = true; // stay spun up
        // mPeriodicIO.SHOOT = false;
        // mPeriodicIO.FENDER = false;
        // mPeriodicIO.SPIT = false;

        // mClimbMode = false;

        System.out.println("Set initial teleop states!");
    }

    /* Superstructure getters for action and goal statuses */
    // get actions
    // public boolean getIntaking() {
    //     return mPeriodicIO.INTAKE;
    // }

    // public boolean getReversing() {
    //     return mPeriodicIO.REVERSE;
    // }


    // included to continue logging while disabled
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mRoll = mPigeon.getRoll().getDegrees();
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "SUPERSTRUCTURE_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");
        headers.add("dt");
        headers.add("gyro roll");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(mPeriodicIO.timestamp);
        items.add(mPeriodicIO.dt);
        // items.add(mPeriodicIO.INTAKE ? 1.0 : 0.0);
        // items.add(mPeriodicIO.REVERSE ? 1.0 : 0.0);
        // items.add(mPeriodicIO.REJECT ? 1.0 : 0.0);
        items.add(mPigeon.getRoll().getDegrees());

        // send data to logging storage
        mStorage.addData(items);
    }



}
