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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        private int maxArmPosition = 0;
        private boolean openingClaw;

        private boolean mEngage = false;
        private boolean fromBack;

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
     * --> 0 to set grip upright
     * --> 90 to set grip sideways
     * --> 180 to flip upsidown
     * --> 270 to set grip sideways other way
     * 
     * - press START button to bring arm and pivot down
     * - BACK/MENU/2 screens button button to getZero
     * 
     * 
     * 
     */

    public void updateOperatorCommands() {

        /* MANUALLY CONTROL PIVOT */
        if (mControlBoard.getOperatorLeftThrottle() > 0.4) {
            System.out.println(mPivot.getPivotDemand() / Constants.PivotConstants.oneDegreeOfroation);
            if ((mPivot.getPivotDemand() / Constants.PivotConstants.oneDegreeOfroation) > 0) {
                mPivot.changePivPosition(mControlBoard.getOperatorLeftThrottle() * -1);
                System.out.println("down");
            }
        } else if (mControlBoard.getOperatorLeftThrottle() < -0.4) {
            System.out.println(mPivot.getPivotDemand() / Constants.PivotConstants.oneDegreeOfroation);
            if ((mPivot.getPivotDemand() / Constants.PivotConstants.oneDegreeOfroation) < 120) {
                mPivot.changePivPosition(mControlBoard.getOperatorLeftThrottle() * -1);
                System.out.println("up");
            }
        }
        /* MANUALLY CONTROL ARM */
        else if (mControlBoard.getOperatorLeftYaw() > 0.4) {
            System.out.println(mArm.getArmPosition());
            if (mArm.getArmPosition() < mPeriodicIO.maxArmPosition) {
                mArm.changeArmPosition(mControlBoard.getOperatorLeftYaw() * 2000);
                System.out.println("extend");
            }
        } else if (mControlBoard.getOperatorLeftYaw() < -0.4) {
            System.out.println(mArm.getArmPosition());
            if (mArm.getArmPosition() > 0) {
                mArm.changeArmPosition(mControlBoard.getOperatorLeftYaw() * 2000);
                System.out.println("retract");
            }
        }

        /* ARM AND PIVOT */
        else {

            if (mControlBoard.getArmDown()) {
                mPeriodicIO.settingDown = true;
                mArm.setArmDown();
                mPeriodicIO.maxArmPosition = 0;
                mPeriodicIO.canControlArmManually = false;
            } else if (mControlBoard.getPickup()) {
                mArm.setArmDown();
                mPeriodicIO.settingPickup = true;
                mPeriodicIO.canControlArmManually = false;
                mPeriodicIO.maxArmPosition = Constants.ArmConstants.kPickupTravelDistance + 20000;
            } else if (mControlBoard.getHybrid()) {
                mArm.setArmDown();
                mPeriodicIO.settingHybrid = true;
                mPeriodicIO.canControlArmManually = false;
                mPeriodicIO.maxArmPosition = Constants.ArmConstants.kHybridTravelDistance;
            } else if (mControlBoard.getMid()) {
                mArm.setArmDown();
                mPeriodicIO.settingMid = true;
                mPeriodicIO.canControlArmManually = false;
                mPeriodicIO.maxArmPosition = Constants.ArmConstants.kMidTravelDistance;
            } else if (mControlBoard.getHigh()) {
                mArm.setArmDown();
                mPeriodicIO.settingHigh = true;
                mPeriodicIO.canControlArmManually = true;
                mPeriodicIO.maxArmPosition = Constants.ArmConstants.kHighTravelDistance;
            }

            if (mPeriodicIO.settingDown) {
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

            // if (mControlBoard.autoTest()) {
            // mSwerve.drive(new Translation2d(mLimelight.getDrivingAdjust(), 0),
            // mLimelight.getSteeringAdjust(),
            // false, true);
            // }

            // hold 3 lines button to pull in arm
            // if (mControlBoard.getZero()) {
            // mArm.resetArmPosition();
            // mPivot.resetPivotPosition();
            // mClaw.zeroSensors();
            // }

            // if ((mControlBoard.getOperatorRightThrottle() > 0.4)
            // || (mControlBoard.getOperatorRightThrottle() < -0.4)) {
            // mClaw.setPivotOpenLoop(mControlBoard.getOperatorRightThrottle() / 2);
            // } else {
            // mClaw.stopPivot();
            // }

            System.out.println(mClaw.getCanCoder());
            System.out.println(mClaw.getPivotDemand() + "&&" + mClaw.getPivotPosition());
            if (mControlBoard.operator.getController().getPOV() == 90) {

                mClaw.setPivotPosition(Constants.ClawConstants.piv_90Rotation);

            } else if (mControlBoard.operator.getController().getPOV() == 0) {

                mClaw.setPivotPosition(Constants.ClawConstants.piv_ZeroRotation);

            } else if (mControlBoard.operator.getController().getPOV() == 180) {

                mClaw.setPivotPosition(Constants.ClawConstants.piv_180Rotation);

            }
            // } else if ((mControlBoard.getOperatorRightYaw() > 0.4)
            // || (mControlBoard.getOperatorRightYaw() < -0.4)) {
            // mClaw.setGripOpenLoop(mControlBoard.getOperatorRightYaw() / 2);

            if (mControlBoard.getZero()) {
                mArm.pullArmIntoZero();
            }

            // right bumper
            if (mControlBoard.getGrip()) {
                mClaw.closeGrip();
            }
            // left bumper
            else if (mControlBoard.getRelease()) {

                mClaw.openGrip();

            } else {
                mClaw.stopGrip();
                // mClaw.stop();
            }
        }
    }

    /*** RUMBLE OPERATOR CONTROLLERS ***/
    public void updateRumble() {
        mControlBoard.setOperatorRumble(false);
    }

    public void pivUpAuto() {

        mPivot.setPivotForAutoHigh();

    }

    public void armExtendHighAuto() {
        mClaw.setPivotPosition(0);
        mArm.setAutoExtendForHigh();

    }

    public void dropConeAuto() {

        mPeriodicIO.openingClaw = true;

    }

    public void getClawOpen() {
        if (mPeriodicIO.openingClaw) {
            mClaw.openGrip();
        }
        if (mClaw.getLimitSwitch()) {
            mPeriodicIO.openingClaw = false;
        }
    }

    public void pivPickupAuto() {

        mPivot.setPivotForPickup();
        mArm.setExtendForPickup();

    }

    public void pivDownAuto() {
        mArm.setArmDown();
        mPivot.setPivotDown();
    }

    public void clampCube() {

        mClaw.closeGrip();

    }

    public void pullArmInAuto() {
        mArm.setArmDown();
    }

    public void pullPivInAuto() {
        mPivot.setPivotDown();
    }

    public void engageChargeStation(boolean fromBack) {
        mPeriodicIO.fromBack = fromBack;
        mPeriodicIO.mEngage = true;

        // if (mPigeon.getRoll().getDegrees() > 3) {
        // mSwerve.drive(new Translation2d(.2, 0), 0, true, false);
        // engageChargeStation();
        // SmartDashboard.putBoolean("ChargeStation", false);
        // SmartDashboard.putBoolean("going forwards balance", false);
        // } else if (mPigeon.getRoll().getDegrees() < -3) {
        // mSwerve.drive(new Translation2d(-.2, 0), 0, true, false);
        // engageChargeStation();
        // SmartDashboard.putBoolean("ChargeStation", false);
        // SmartDashboard.putBoolean("going forwards balance", false);
        // } else {
        // mSwerve.setLocked(true);
        // SmartDashboard.putBoolean("ChargeStation", true);
        // // SmartDashboard.putBoolean("going forwards balance", true);
        // // SmartDashboard.putNumber("Pitch", mPigeon.getPitch().getDegrees());
        // }
    }

    public boolean[] getAutoBalance() {
        boolean[] booleanArray = { mPeriodicIO.mEngage, mPeriodicIO.fromBack };
        return booleanArray;
    }

    /*** UPDATE STATUS LEDS ON ROBOT ***/
    public void updateLEDs() {
        if (mLEDs.getUsingSmartdash()) {
            return;
        }

        State mState = State.OFF;

        if (hasEmergency) {
            mState = State.EMERGENCY;
        } else {
            if (mClaw.mPeriodicIO.grip_motor_velocity < -0.2) {
                mState = State.FLASHING_ORANGE;
            } else if (mClaw.getWantRelease()) {
                mState = State.FLASHING_CYAN;
            } else if (Timer.getFPGATimestamp() > 135) {
                mState = State.FLASHING_PINK;
            } else if (mPeriodicIO.mEngage) {
                mState = State.SOLID_PINK;
            } else {
                mState = State.SOLID_YELLOW;
            }
        }
        // mState = State.FLASHING_CYAN;

        mLEDs.applyStates(mState);
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

    }

    /* Initial states for superstructure for teleop */
    public void setInitialTeleopStates() {

        // mClimbMode = false;

        System.out.println("Set initial teleop states!");
    }

    // included to continue logging while disabled
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mRoll = mPigeon.getRoll().getDegrees();
        // mSwerve.readPeriodicInputs();
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
