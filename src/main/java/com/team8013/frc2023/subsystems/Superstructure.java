package com.team8013.frc2023.subsystems;

import com.fasterxml.jackson.databind.deser.std.MapEntryDeserializer;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.controlboard.ControlBoard;
import com.team8013.frc2023.drivers.Pigeon;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team8013.frc2023.loops.ILooper;
import com.team8013.frc2023.loops.Loop;
import com.team8013.frc2023.subsystems.LEDs.State;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Superstructure extends Subsystem {

    // superstructure instance
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    /*** REQUIRED INSTANCES ***/
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final ClawV2 mClaw = ClawV2.getInstance();
    private final PivotV2 mPivot = PivotV2.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();
    private final Pigeon mPigeon = Pigeon.getInstance();

    // robot state
    // private final RobotState mRobotState = RobotState.getInstance();

    private double rollAdjust = 0;
    private boolean hasBeenLevel = false;

    // Timer autoBalanceTimer = new Timer();

    PIDController m_BalancePid;

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
        private boolean placingCube = false;
        private boolean settingDoubleSubstation = false;
        private boolean settingReadyPosition = false;
        private boolean canControlArmManually = true;
        public double maxArmPosition = 0;
        public boolean openingClaw;

        // AUTO
        public boolean mEngage = false;
        public boolean fromBack;
        public boolean clampClawAuto;
        public boolean hasClosedGrip;
        public boolean wantDropPieceAuto;

        // time measurements
        public double timestamp;
        public double dt;

        // LEDs
        public boolean wantedYellow;
        public boolean wantedBlue;
    }

    // // OUTPUTS
    double mRoll = 0;

    // // aiming parameter vars
    // private Optional<AimingParameters> real_aiming_params_ = Optional.empty();
    // private int mTrackId = -1;
    // private double mTargetAngle = 0.0;
    // public double mCorrectedDistanceToTarget = 0.0;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                m_BalancePid = new PIDController(Constants.AutoConstants.balance_kP, Constants.AutoConstants.balance_kI,
                        Constants.AutoConstants.balance_kD);
                m_BalancePid.setTolerance(Constants.AutoConstants.balance_PositionTolerance);
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
     * - press right trigger to move pivot up
     * - press left trigger to move pivot down
     * 
     * - press right bumper to grip
     * - press left bumper to ungrip
     * 
     * - hold both bumpers for 1 sec to go to double substation
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
     * --> 90 to set grip sideways or for cone
     * --> 180 to flip upsidown
     * --> 270 to set grip sideways other way or for cube
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
            // System.out.println(mPivot.getPivotDemand() /
            // Constants.PivotConstants.oneDegreeOfroation);
            if ((mPivot.getPivotDemand() / Constants.PivotConstants.oneDegreeOfroation) > 0) {
                mPivot.changePivPosition(-0.3 * mControlBoard.getOperatorLeftThrottle());
                System.out.println("down");
            }
        } else if (mControlBoard.getOperatorLeftThrottle() < -0.4) {
            // System.out.println(mPivot.getPivotDemand() /
            // Constants.PivotConstants.oneDegreeOfroation);
            if ((mPivot.getPivotDemand() / Constants.PivotConstants.oneDegreeOfroation) < (2048 * 120)) {
                mPivot.changePivPosition(-0.3 * mControlBoard.getOperatorLeftThrottle());
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
            } else if (mControlBoard.getWantDoubleSubstation()) {
                mArm.setArmDown();
                mPeriodicIO.settingDoubleSubstation = true;
                mPeriodicIO.canControlArmManually = false;
                mPeriodicIO.maxArmPosition = Constants.ArmConstants.kDoubleSubstationTravelDistance;
            } else if (mControlBoard.getWantReadyPosition()) {
                mArm.setArmDown();
                mPeriodicIO.settingReadyPosition = true;
                mPeriodicIO.canControlArmManually = false;
                mPeriodicIO.maxArmPosition = 10;
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
            if (mPeriodicIO.settingDoubleSubstation) {
                if (mArm.isIn()) {
                    mPivot.setPivotForDoubleSubstation();
                }
                if (mPivot.canExtendArm(Constants.PivotConstants.kDoubleSubstationTravelDistance)) {
                    mArm.setExtendForDoubleSubstation();
                    mPeriodicIO.settingDoubleSubstation = false;
                    mPeriodicIO.canControlArmManually = true;
                }
            }
            if (mPeriodicIO.settingReadyPosition) {
                if (mArm.isIn()) {
                    mPivot.setPivotForReadyPosition();
                }
                if (mPivot.canExtendArm(Constants.PivotConstants.kReadyPositionTravelDistance)) {
                    mArm.setArmDown();
                    mPeriodicIO.settingReadyPosition = false;
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

            // TODO: uncomment this to find pivot kG
            // if ((mControlBoard.getOperatorRightThrottle() > 0.4)
            // || (mControlBoard.getOperatorRightThrottle() < -0.4)) {
            // mClaw.setPivotOpenLoop(mControlBoard.getOperatorRightThrottle() / 2);
            // } else {
            // mClaw.stopPivot();
            // }

            System.out.println(mClaw.getCanCoder());
            System.out.println(mClaw.getPivotDemand() + "&&" + mClaw.getPivotPosition());
            // if (mControlBoard.operator.getController().getPOV() == 90) {

            // mClaw.setPivotPosition(Constants.ClawConstants.piv_90Rotation);

            // } else
            if (mControlBoard.operator.getController().getPOV() == 0) {

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
            }
        }
    }

    /*** RUMBLE OPERATOR CONTROLLERS ***/
    public void updateRumble() {
        mControlBoard.setOperatorRumble(false);
    }

    /* AUTO COMMANDS */

    /**
     * @return this sets the pivot and arm to high and drops the game piece, then
     *         returns to the down position
     */
    public void settingHighToDownAuto() {
        mArm.setArmDown();
        mPeriodicIO.settingHigh = true;
    }

    public void settingCubeDropHigh() {
        mClaw.mPeriodicIO.wantedClosing = false;
        mPeriodicIO.settingPickup = false;
        mPeriodicIO.clampClawAuto = false;
        mPeriodicIO.wantDropPieceAuto = true;
        mArm.setArmDown();
        mPeriodicIO.placingCube = true;
    }

    /**
     * @return this sets the pivot and arm to hybrid and drops the game piece, then
     *         returns to the down position
     */
    public void settingHybridToDownAuto() {
        mArm.setArmDown();
        mPeriodicIO.settingHybrid = true;
    }

    /**
     * @return sets the arm and pivot to the pickup position
     */
    public void settingPickupAuto() {
        if ((Math.abs(mClaw.mPeriodicIO.pivot_motor_position) > 120)
                && (Math.abs(mClaw.mPeriodicIO.pivot_motor_position) < 210)) {
            mClaw.setPivotPosition(Constants.ClawConstants.piv_180Rotation);
        } else {
            mClaw.setPivotPosition(Constants.ClawConstants.piv_ZeroRotation);
        }
        mArm.setArmDown();
        mPeriodicIO.settingPickup = true;
    }

    /**
     * @return when called, this clamps the claw in auto (after the extended pickup
     *         position is reached)
     *         and pulls the arm back into the down position after the game piece is
     *         clamped
     */
    public void clampClawAuto() {
        mPeriodicIO.clampClawAuto = true;
    }

    /**
     * @return when this is called it allows the robot to drop the game piece, so
     *         call it after the robot has reached its target position
     */
    public void wantDropPieceAuto() {
        mPeriodicIO.wantDropPieceAuto = true;
    }

    public void openOpenClaw() {
        mPeriodicIO.openingClaw = true;
    }

    /**
     * @return this sets the arm and pivot to the down position
     */
    public void setDownAuto() {
        mPeriodicIO.settingDown = true;
    }

    /**
     * @return if the claw starts backwards during auto then it keeps it backwards
     *         otherwise it sets the claw pivot to 0 and extends the arm out
     */
    private void armExtendHighAuto() {
        // if ((Math.abs(mClaw.mPeriodicIO.pivot_motor_position) > 120)
        // && (Math.abs(mClaw.mPeriodicIO.pivot_motor_position) < 210)) {
        // mClaw.setPivotPosition(Constants.ClawConstants.piv_180Rotation);
        // } else {
        // mClaw.setPivotPosition(Constants.ClawConstants.piv_ZeroRotation);
        // }
        mArm.setAutoExtendForHigh();
    }

    // this method is called periodically through auto
    public void autoPeriodic() {
        // Claw open
        if (mPeriodicIO.openingClaw) {
            mClaw.openGrip();
            mPeriodicIO.hasClosedGrip = false;
        }
        if (mClaw.getLimitSwitch()) {
            mPeriodicIO.openingClaw = false;
        }

        // high drop setting
        if (mPeriodicIO.settingHigh) {
            if (mArm.isIn()) {
                mPivot.setPivotForAutoHigh();
                mArm.setArmDown();
            }

            if (mPivot.canExtendArm(Constants.PivotConstants.kAutoHighTravelDistance)) {
                armExtendHighAuto();
            }

            if ((mArm.canDropCone(Constants.ArmConstants.kAutoHighTravelDistance)) && (mPeriodicIO.wantDropPieceAuto)) {
                mPeriodicIO.openingClaw = true;
                if (mClaw.getLimitSwitch()) {
                    mPeriodicIO.openingClaw = false;
                    mPeriodicIO.settingHigh = false;
                    mPeriodicIO.settingDown = true;
                    mPeriodicIO.wantDropPieceAuto = false;
                }
            }
        }

        // hybrid drop setting
        if (mPeriodicIO.settingHybrid) {
            if (mArm.isIn()) {
                mPivot.setPivotForHybrid();
            }

            if (mPivot.canExtendArm(Constants.PivotConstants.kHybridTravelDistance)) {
                mArm.setExtendForHybrid();
            }

            if (mArm.canDropCone(Constants.ArmConstants.kHybridTravelDistance)) {
                mPeriodicIO.openingClaw = true;
                if (mClaw.getLimitSwitch()) {
                    mPeriodicIO.openingClaw = false;
                    mPeriodicIO.settingHybrid = false;
                    mPeriodicIO.settingDown = true;
                }
            }
        }

        if (mPeriodicIO.placingCube) {

            if (mArm.isIn()) {
                mPivot.setPivotForAutoHigh();
            }

            if (mPivot.canExtendArm(Constants.PivotConstants.kAutoHighTravelDistance)) {
                armExtendHighAuto();
            }

            mPeriodicIO.clampClawAuto = false;

            if ((mArm.canDropCone(Constants.ArmConstants.kAutoHighTravelDistance))) {
                mClaw.openGrip();

                if (mClaw.getLimitSwitch()) {
                    mPeriodicIO.openingClaw = false;
                    mPeriodicIO.placingCube = false;
                    mPeriodicIO.settingDown = true;
                    mPeriodicIO.wantDropPieceAuto = false;
                }
            }
        }

        // down setting
        if (mPeriodicIO.settingDown) {
            mArm.setArmDown();
            mPeriodicIO.settingPickup = false;

            if (mArm.isIn()) {
                mPivot.setPivotForReadyPosition();
                mPeriodicIO.settingDown = false;
            }
        }

        // pickup piece setting
        if ((mPeriodicIO.settingPickup) && (!mPeriodicIO.settingDown)) {
            if (mArm.isIn()) {
                mPivot.setPivotForPickup();
            }
            if (mPivot.canExtendArm(Constants.PivotConstants.kPickupTravelDistance)) {
                mArm.setExtendForPickup();
            }
            // wait for routine to call clamp, clamp once, then end pickup by calling
            // setting down
            if ((mPeriodicIO.clampClawAuto) && (mArm.canDropCone(Constants.ArmConstants.kPickupTravelDistance))) {
                // if (!mPeriodicIO.hasClosedGrip) {
                mClaw.closeGrip();
                // mPeriodicIO.hasClosedGrip = true;
                if (!mClaw.mPeriodicIO.wantedClosing) {
                    mPeriodicIO.settingPickup = false;
                    mPeriodicIO.settingDown = true;
                }
            }

        }

    }

    // public void pivPickupAuto() {

    // mPivot.setPivotForPickup();
    // mArm.setExtendForPickup();

    // }

    // public void pivDownAuto() {
    // mArm.setArmDown();
    // mPivot.setPivotDown();
    // }

    // public void pullArmInAuto() {
    // mArm.setArmDown();
    // }

    // public void pullPivInAuto() {
    // mPivot.setPivotDown();
    // }

    // public void dropConeAuto() {

    // mPeriodicIO.openingClaw = true;

    // }

    public void engageChargeStation(boolean fromBack) {
        mPeriodicIO.fromBack = fromBack;
        mPeriodicIO.mEngage = true;
    }

    public boolean[] getAutoBalance() {
        boolean[] booleanArray = { mPeriodicIO.mEngage, mPeriodicIO.fromBack };
        return booleanArray;
    }

    // uses pid to engage
    public void autoBalancePID() {
        if (mPeriodicIO.mEngage) {
            m_BalancePid.setSetpoint(0);

            double output = MathUtil.clamp(m_BalancePid.calculate(getAdjustedRoll()),
                    Constants.AutoConstants.balance_kMinOutput,
                    Constants.AutoConstants.balance_kMaxOutput);

            SmartDashboard.putNumber("auto balance output", output);

            if (!m_BalancePid.atSetpoint()) {
                mSwerve.setLocked(false);
                mSwerve.drive(new Translation2d(output, 0), 0, true, true);
            } else {
                mSwerve.setLocked(true);
            }
        }
    }

    // if gyro is slanted drive until it changes to the negative angle, then drive
    // slowly backward until it levels
    public void autoBalanceNonPID() {
        if (mPeriodicIO.mEngage) {

            // if it is slanted, go at a higher speed until it goes past level
            if ((getAdjustedRoll() > Constants.AutoConstants.firstAngle) && (!mPeriodicIO.fromBack)) {
                mSwerve.drive(new Translation2d(Constants.AutoConstants.firstSpeed, 0), 0, true, false);
            } else if ((getAdjustedRoll() < -Constants.AutoConstants.firstAngle) && (mPeriodicIO.fromBack)) {
                mSwerve.drive(new Translation2d(-Constants.AutoConstants.firstSpeed, 0), 0, true, false);
            }
            // drive slower back until level
            else if ((getAdjustedRoll() < -Constants.AutoConstants.secondAngle) && (!mPeriodicIO.fromBack)) {
                mSwerve.setLocked(false);
                mSwerve.drive(new Translation2d(-Constants.AutoConstants.secondSpeed, 0), 0, true, false);
            } else if ((getAdjustedRoll() > Constants.AutoConstants.secondAngle) && (mPeriodicIO.fromBack)) {
                mSwerve.setLocked(false);
                mSwerve.drive(new Translation2d(Constants.AutoConstants.secondSpeed, 0), 0, true, false);
            }
            // when 'level' lock the wheels
            else {
                mSwerve.drive(new Translation2d(0, 0), 0, true, false);
            }

            // SmartDashboard.putBoolean("Balance From Back", mPeriodicIO.fromBack);
            // SmartDashboard.putBoolean("Swerve Locked", mSwerve.getLocked());
        }
    }

        // if gyro is slanted drive until it changes to the negative angle, then drive
    // slowly backward until it levels
    public void autoBalanceNonPIDSlowSpeedOverTime() {
        if (mPeriodicIO.mEngage) {

            // if it is slanted, go at a higher speed until it goes past level
            if ((getAdjustedRoll() > Constants.AutoConstants.firstAngle) && (!mPeriodicIO.fromBack)) {
                if(hasBeenLevel){
                    mSwerve.drive(new Translation2d(Constants.AutoConstants.secondSpeed, 0), 0, true, false);
                }
                else{
                mSwerve.drive(new Translation2d(Constants.AutoConstants.firstSpeed, 0), 0, true, false);
                }
            } else if ((getAdjustedRoll() < -Constants.AutoConstants.firstAngle) && (mPeriodicIO.fromBack)) {
                if(hasBeenLevel){
                    mSwerve.drive(new Translation2d(-Constants.AutoConstants.secondSpeed, 0), 0, true, false);
                }
                else{
                mSwerve.drive(new Translation2d(-Constants.AutoConstants.firstSpeed, 0), 0, true, false);
                }
            }
            // drive slower back until level
            else if ((getAdjustedRoll() < -Constants.AutoConstants.secondAngle) && (!mPeriodicIO.fromBack)) {
                mSwerve.drive(new Translation2d(-Constants.AutoConstants.secondSpeed, 0), 0, true, false);
            } else if ((getAdjustedRoll() > Constants.AutoConstants.secondAngle) && (mPeriodicIO.fromBack)) {
                mSwerve.drive(new Translation2d(Constants.AutoConstants.secondSpeed, 0), 0, true, false);
            }
            // when 'level' lock the wheels
            else {
                if (!hasBeenLevel){
                    hasBeenLevel = true;
                }
                mSwerve.drive(new Translation2d(0, 0), 0, true, false);
            }

            // SmartDashboard.putBoolean("Balance From Back", mPeriodicIO.fromBack);
            // SmartDashboard.putBoolean("Swerve Locked", mSwerve.getLocked());
        }
    }

    public void zeroRollInit() {
        rollAdjust = mPigeon.getRoll().getDegrees();
        SmartDashboard.putNumber("PIGEON roll adjust", rollAdjust);
    }

    public double getAdjustedRoll() {
        return mPigeon.getRoll().getDegrees() - rollAdjust;
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
            if (mControlBoard.getWantCone()) {
                mState = State.FLASHING_YELLOW;
                mPeriodicIO.wantedYellow = true;
                mPeriodicIO.wantedBlue = false;
                mLEDs.applyStates(mState);
                return;
            } else if (mControlBoard.getWantCube()) {
                mState = State.FLASHING_PURPLE;
                mPeriodicIO.wantedBlue = true;
                mPeriodicIO.wantedYellow = false;
                mLEDs.applyStates(mState);
                return;
            } else if (mClaw.mPeriodicIO.wantedClosing) {
                mState = State.SOLID_ORANGE;
            }
            // } else if (Timer.getFPGATimestamp() > 135) {
            // // mState = State.SOLID_PINK;
            // }
            else if (mPeriodicIO.wantedBlue) {
                mState = State.SOLID_PURPLE;
            } else if (mPeriodicIO.wantedYellow) {
                mState = State.SOLID_YELLOW;
            } else {
                mState = State.OFF;
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
        mSwerve.drive(new Translation2d(0, 0), 0, true, false);
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
