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
import com.team8013.frc2023.subsystems.LEDs.State;
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
    private final ClawV2 mClaw = ClawV2.getInstance();
    private final Pivot mPivot = Pivot.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();
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

        // private boolean GRABBED = false; // run the intake to pick up cargo

        private boolean settingDown = false;
        private boolean settingPickup = false;
        private boolean settingHybrid = false;
        private boolean settingMid = false;
        private boolean settingHigh = false;
        private boolean canControlArmManually = true;

        // time measurements
        public double timestamp;
        public double dt;
    }

    // // OUTPUTS
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

                // updateVisionAimingParameters();

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

    // public enum WantedAction {
    // NONE, STORE, PICKUP, HYBRID, MID, HIGH
    // }

    /***
     * CONTAINER FOR OPERATOR COMMANDS CALLING SUPERSTRUCTURE ACTIONS
     * Jack, feel free to change this!
     * 
     * 
     * - hold right trigger to
     * - hold left trigger to
     * 
     * - press right bumper to grip
     * - press left bumper to ungrip
     * 
     * - press A to getPickup
     * - press Y to getHigh
     * - press B to getHybrid
     * - press X to getMid
     * 
     * - left joystick yaw to move arm in and out manually
     * - left joystick throttle to move pivot up and down manually
     * 
     * - right joystick yaw to grip claw manually
     * - right joystick throttle to pivot claw manually
     * 
     * - Use dpad to
     * --> 0 to
     * --> 180 to
     * 
     * - press START button / 3 lines button to getZero
     * 
     * 
     * 
     */
    public void putArmAtZero() {
        mArm.pullArmIntoZero();
    }

    public void updateOperatorCommands() {

        /* MANUALLY CONTROL PIVOT */
        if ((mControlBoard.getOperatorLeftThrottle() < -0.4) ||
                (mControlBoard.getOperatorLeftThrottle() > 0.4)) {
            mPivot.setPivotOpenLoop(mControlBoard.getOperatorLeftThrottle());
        }
        /* MANUALLY CONTROL ARM */
        else if ((mControlBoard.getOperatorLeftYaw() < -0.4) ||
                (mControlBoard.getOperatorLeftYaw() > 0.4)) {
            mArm.setArmOpenLoop(mControlBoard.getOperatorLeftYaw());
            /* ARM AND PIVOT */
        } else {

            if (mControlBoard.getArmDown()) {
                mPeriodicIO.settingDown = true;
                mArm.setArmDown();
                mPeriodicIO.canControlArmManually = false;
                System.out.println("working");
            } else if (mControlBoard.getPickup()) {
                mArm.setArmDown();
                mPeriodicIO.settingPickup = true;
                mPeriodicIO.canControlArmManually = false;
            } else if (mControlBoard.getHybrid()) {
                mArm.setArmDown();
                mPeriodicIO.settingHybrid = true;
                mPeriodicIO.canControlArmManually = false;
            } else if (mControlBoard.getMid()) {
                mArm.setArmDown();
                mPeriodicIO.settingMid = true;
                mPeriodicIO.canControlArmManually = false;
            } else if (mControlBoard.getHigh()) {
                mArm.setArmDown();
                mPeriodicIO.settingHigh = true;
                mPeriodicIO.canControlArmManually = true;
            }

            if (mPeriodicIO.settingDown) {
                System.out.println("working2");
                if (mArm.isIn()) {
                    mPivot.setPivotDown();
                    mPeriodicIO.settingDown = false;
                }
            }
            if (mPeriodicIO.settingPickup) {
                if (mArm.isIn()) {
                    mPivot.setPivotForPickup();
                }
                if (mPivot.canExtendArm(Constants.PivotConstants.kPickupTravelDistance)) {
                    mArm.setExtendForPickup();
                    mPeriodicIO.settingPickup = false;
                    mPeriodicIO.canControlArmManually = true;
                }
            }
            if (mPeriodicIO.settingHybrid) {
                if (mArm.isIn()) {
                    mPivot.setPivotForHybrid();
                }
                if (mPivot.canExtendArm(Constants.PivotConstants.kHybridTravelDistance)) {
                    mArm.setExtendForHybrid();
                    mPeriodicIO.settingHybrid = false;
                    mPeriodicIO.canControlArmManually = true;
                }
            }
            if (mPeriodicIO.settingMid) {
                if (mArm.isIn()) {
                    mPivot.setPivotForMid();
                }
                if (mPivot.canExtendArm(Constants.PivotConstants.kMidTravelDistance)) {
                    mArm.setExtendForMid();
                    mPeriodicIO.settingMid = false;
                    mPeriodicIO.canControlArmManually = true;
                }
            }
            if (mPeriodicIO.settingHigh) {
                if (mArm.isIn()) {
                    mPivot.setPivotForHigh();
                }
                if (mPivot.canExtendArm(Constants.PivotConstants.kHighTravelDistance)) {
                    mArm.setExtendForHigh();
                    mPeriodicIO.settingHigh = false;
                    mPeriodicIO.canControlArmManually = true;
                }
            }

            System.out.println(mArm.getArmPosition());

            // if (mControlBoard.autoTest()) {
            // mSwerve.drive(new Translation2d(mLimelight.getDrivingAdjust(), 0),
            // mLimelight.getSteeringAdjust(),
            // false, true);
            // }

            // hold 3 lines button to pull in arm
            if (mControlBoard.getZero()) {
                mArm.resetClimberPosition();
                mPivot.resetPivotPosition();
                mClaw.zeroSensors();
            }

            if ((mControlBoard.getOperatorRightThrottle() > 0.4)
                    || (mControlBoard.getOperatorRightThrottle() < -0.4)) {
                mClaw.setPivotOpenLoop(mControlBoard.getOperatorRightThrottle() / 2);
            } else if ((mControlBoard.getOperatorRightYaw() > 0.4)
                    || (mControlBoard.getOperatorRightYaw() < -0.4)) {
                mClaw.setGripOpenLoop(mControlBoard.getOperatorRightYaw() / 2);
            } else if (mControlBoard.getGrip()) {
                mClaw.closeGrip();
            } else if (mControlBoard.getRelease()) {
                mClaw.openGrip();
            } else {
                mClaw.stop();
                // mClaw.setPivotPosition(mControlBoard.operator.getController().getPOV());
            }
        }
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
    // Optional<AimingParameters> aiming_params =
    // RobotState.getInstance().getAimingParameters(mTrackId,
    // Constants.VisionConstants.kMaxGoalTrackAge);
    // if (aiming_params.isPresent()) {
    // return aiming_params;
    // } else {
    // Optional<AimingParameters> default_aiming_params =
    // RobotState.getInstance().getDefaultAimingParameters();
    // return default_aiming_params;
    // }
    // }

    // /*** UPDATE VISION AIMING PARAMETERS FROM GOAL TRACKING ***/
    // public void updateVisionAimingParameters() {
    // // get aiming parameters from either vision-assisted goal tracking or
    // // odometry-only tracking
    // real_aiming_params_ = getRealAimingParameters();

    // // predicted pose and target
    // Pose2d predicted_field_to_vehicle = mRobotState
    // .getPredictedFieldToVehicle(Constants.VisionConstants.kLookaheadTime);
    // Pose2d predicted_vehicle_to_goal = predicted_field_to_vehicle.inverse()
    // .transformBy(real_aiming_params_.get().getFieldToGoal());

    // // update align delta from target and distance from target
    // mTrackId = real_aiming_params_.get().getTrackId();
    // mTargetAngle =
    // predicted_vehicle_to_goal.getTranslation().direction().getRadians() +
    // Math.PI;

    // // send vision aligning target delta to swerve
    // mSwerve.acceptLatestGoalTrackVisionAlignGoal(mTargetAngle);

    // // update distance to target
    // if (mLimelight.hasTarget() &&
    // mLimelight.getLimelightDistanceToTarget().isPresent()) {
    // mCorrectedDistanceToTarget = mLimelight.getLimelightDistanceToTarget().get();
    // } else {
    // mCorrectedDistanceToTarget =
    // predicted_vehicle_to_goal.getTranslation().norm();
    // }
    // }

    /*** UPDATE STATUS LEDS ON ROBOT ***/
    public void updateLEDs() {
        if (mLEDs.getUsingSmartdash()) {
            return;
        }

        State frontState = State.OFF;
        State backState = State.OFF;
        if (hasEmergency) {
            State topState = State.EMERGENCY;
            State bottomState = State.EMERGENCY;
        } else {
            // if (!mClimbMode) {
            // if (getBallCount() == 2) {
            // bottomState = State.SOLID_GREEN;
            // } else if (getBallCount() == 1) {
            // bottomState = State.SOLID_CYAN;
            // } else {
            // bottomState = State.SOLID_ORANGE;
            // }
            // if (getWantsSpit()) {
            // topState = State.SOLID_ORANGE;
            // } else if (getWantsFender()) {
            // topState = State.SOLID_CYAN;
            // } else if (mPeriodicIO.SHOOT) {
            // topState = State.FLASHING_PINK;
            // } else if (isAimed()) {
            // topState = State.FLASHING_GREEN;
            // } else if (hasTarget()) {
            // topState = State.SOLID_PURPLE;
            // } else {
            // topState = State.SOLID_ORANGE;
            // }
            // } else {
            // if (mOpenLoopClimbControlMode) {
            // topState = State.SOLID_YELLOW;
            // bottomState = State.SOLID_YELLOW;
            // } else if (mAutoTraversalClimb) {
            // topState = State.FLASHING_ORANGE;
            // bottomState = State.FLASHING_ORANGE;
            // } else if (mAutoHighBarClimb) {
            // topState = State.FLASHING_CYAN;
            // bottomState = State.FLASHING_CYAN;
            // } else {
            // topState = State.SOLID_PINK;
            // bottomState = State.SOLID_PINK;
            // }
            // }
        }
        frontState = State.FLASHING_CYAN;
        backState = State.FLASHING_PINK;

        mLEDs.applyStates(frontState, backState);
    }

    /***
     * UPDATE SUBSYSTEM STATES + SETPOINTS AND SET GOALS
     * 
     * 1. updates wanted actions for intake and indexer subsystems based on
     * requested superstructure action
     * 2. updates shooter and hood setpoint goals from tracked vars
     * 3. set subsystem states and shooting setpoints within subsystems
     * 
     */
    public void setGoals() {
        /* Update subsystem wanted actions and setpoints */

    }

    // // get vision align delta from goal
    // public double getVisionAlignGoal() {
    // return mTargetAngle;
    // }

    // check if our limelight sees a vision target
    public boolean hasTarget() {
        return mLimelight.hasTarget();
    }

    // // checked if we are vision aligned to the target within an acceptable horiz.
    // error
    // public boolean isAimed() {
    // return mLimelight.isAimed();
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
    // return mPeriodicIO.INTAKE;
    // }

    // public boolean getReversing() {
    // return mPeriodicIO.REVERSE;
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
