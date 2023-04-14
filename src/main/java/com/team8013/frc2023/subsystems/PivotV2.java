package com.team8013.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.lib.util.CTREConfigs;

import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotV2 extends Subsystem {

    private static PivotV2 mInstance;

    public static synchronized PivotV2 getInstance() {
        if (mInstance == null) {
            mInstance = new PivotV2();
        }
        return mInstance;
    }

    private final TalonFX mPivot;
    // private final PIDController mPivotPid;

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    // status variable for is enabled
    public boolean mIsEnabled = false;

    public PivotControlState mPivotControlState = PivotControlState.OPEN_LOOP;
    MotionMagicDutyCycle motionMagicDutyCycle;
    DutyCycleOut gravityFeedForward;

    // private boolean mExtendPivot = false;
    // private boolean mPartialExtendPivot = false;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private CANcoder angleEncoder;

    // public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true,
    //         Constants.PivotConstants.kStatorCurrentLimit, 60, .2);

    private PivotV2() {
        angleEncoder = new CANcoder(Ports.PIV_CANCODER, "canivore1");
        configAngleEncoder();

        mPivot = TalonFXFactory.createDefaultTalon(Ports.PIVOT); // TODO:SET THE PORT
        configPivotMotor();

        setBrakeMode(true);

        motionMagicDutyCycle = new MotionMagicDutyCycle(getCANCoderToMotorRotations());
        motionMagicDutyCycle.OverrideBrakeDurNeutral = true;
        motionMagicDutyCycle.EnableFOC = false;

        

    }

    @Override
    public void readPeriodicInputs() {

        mPeriodicIO.pivot_voltage = mPivot.getSupplyVoltage().getValue();
        mPeriodicIO.pivot_current = mPivot.getStatorCurrent().getValue();
        mPeriodicIO.pivot_motor_velocity = mPivot.getVelocity().getValue();
        mPeriodicIO.pivot_motor_position = mPivot.getRotorPosition().getValue();

        mPeriodicIO.pivot_cancoder = getRelativeCancoder();

        // send log data
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        // set status variable for being enabled to true
        mIsEnabled = true;

        // TODO:
        if (Math.abs(getCANCoderToMotorRotations()
                - mPeriodicIO.pivot_motor_position) > Constants.PivotConstants.ticksOffToReset) {
            mPivot.setRotorPosition(getCANCoderToMotorRotations());
        }

        // double feedForward = Constants.PivotConstants.kGravity *
        // Math.cos(mPeriodicIO.pivot_cancoder - 90); // 0 degrees
        // is
        // horisontal for
        // this

        SmartDashboard.putNumber("Pivot Motor", mPeriodicIO.pivot_motor_position);

        switch (mPivotControlState) {
            case OPEN_LOOP:
                mPivot.set(mPeriodicIO.pivot_demand);
                motionMagicDutyCycle.Position = getCANCoderToMotorRotations();
                break;
            case CLOSED_LOOP:
                motionMagicDutyCycle.Position = mPeriodicIO.pivot_demand;
                double feedForward = Math.sin(getRelativeCancoder())*Constants.PivotConstants.kGravity;
                motionMagicDutyCycle.withFeedForward(feedForward);
                mPivot.setControl(motionMagicDutyCycle);

                // , DemandType.ArbitraryFeedForward, feedForward);


                break;
            // case CLOSED_ENCODER: //use new pid controller that uses the encoder for input
            // mNeoMotor.s
            default:
                mPivot.set(0);
                break;
        }

    }

    public synchronized void setBrakeMode(boolean brake) {
        CTREConfigs.armPivotFXConfig().MotorOutput.NeutralMode = (brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        mPivot.getConfigurator().apply(CTREConfigs.armPivotFXConfig());
    }

    public synchronized void resetPivotPosition() {
        mPivot.setRotorPosition(0.0);
    }

    /**
     * @param wantedDemand
     * @return sets the motor percent output to wanted demand (-1,1)
     */
    public void setPivotOpenLoop(double wantedDemand) {
        if (mPivotControlState != PivotControlState.OPEN_LOOP) {
            mPivotControlState = PivotControlState.OPEN_LOOP;
        }
        mPeriodicIO.pivot_demand = MathUtil.clamp(wantedDemand, -1, 1);
    }

    /**
     * @param wantedPositionDegrees degrees you want the pivot to go to
     * @return sets the position to that degrees
     */
    public void setPivotPosition(double wantedPositionDegrees) {
        if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
            mPivotControlState = PivotControlState.CLOSED_LOOP;
        }
        mPeriodicIO.pivot_demand = wantedPositionDegrees * Constants.PivotConstants.oneDegreeOfroation;
    }

    /**
     * @param wantedPositionDelta (wanted position change in degrees)
     * @return Sets the Pivot position to what its at + wantedPosition
     */
    public void changePivPosition(double wantedPositionDelta) {
        if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
            mPivotControlState = PivotControlState.CLOSED_LOOP;

            // mPeriodicIO.pivot_demand = mPeriodicIO.pivot_motor_position;
        }
        mPeriodicIO.pivot_demand = mPeriodicIO.pivot_demand
                + (wantedPositionDelta * Constants.PivotConstants.oneDegreeOfroation);
    }

    // Basically does nothing
    // public void setPivotPosToCancoder() {
    // if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
    // mPivotControlState = PivotControlState.CLOSED_LOOP;
    // }
    // mPeriodicIO.pivot_demand = getRelativeCancoder();
    // }

    /* Pivot METHODS */

    /**
     * 
     * @return true if the pivot is rotated 30 degrees
     */
    public boolean canExtendArm() {
        return Math.abs(mPeriodicIO.pivot_cancoder) > 30;
    }

    /**
     * @param degree the degree that you want the pivot to turn to
     * @return true if the pivot is within 4 degrees of that degree you want it to
     *         turn
     */
    public boolean canExtendArm(double degree) {
        return (Math.abs((Math.abs(mPeriodicIO.pivot_cancoder)
                - Math.abs(degree))) < Constants.PivotConstants.degreesCanExtendArm);
    }

    /**
     * 
     * @return sets the pivot position to a pickup constant
     */
    public void setPivotForPickup() {
        setPivotPosition(Constants.PivotConstants.kPickupTravelDistance);
    }

    /**
     * 
     * @return sets the pivot position to a hybrid constant
     */
    public void setPivotForHybrid() {
        setPivotPosition(Constants.PivotConstants.kHybridTravelDistance);
    }

    /**
     * 
     * @return sets the pivot position to a mid constant
     */
    public void setPivotForMid() {
        setPivotPosition(Constants.PivotConstants.kMidTravelDistance);
    }

    /**
     * 
     * @return sets the pivot position to a highconstant
     */
    public void setPivotForHigh() {
        setPivotPosition(Constants.PivotConstants.kHighTravelDistance);
    }

    /**
     * 
     * @return sets the pivot position to an auto high constant
     */
    public void setPivotForAutoHigh() {
        setPivotPosition(Constants.PivotConstants.kAutoHighTravelDistance);
    }

    /**
     * 
     * @return sets the pivot position to a double substation constant
     */
    public void setPivotForDoubleSubstation() {
        setPivotPosition(Constants.PivotConstants.kDoubleSubstationTravelDistance);
    }

    /**
     * 
     * @return sets the pivot position to a double substation constant
     */
    public void setPivotForReadyPosition() {
        setPivotPosition(Constants.PivotConstants.kReadyPositionTravelDistance);
    }

    /**
     * 
     * @return sets the pivot position to 0, down
     */
    public void setPivotDown() {
        setPivotPosition(0);
    }

    // hold current position on Pivot
    // If Pivot is moving less than 0.5 m/s and Pivot current is greater than
    // constant
    // stator current limit
    // public void maybeHoldCurrentPosition() {
    // if (Util.epsilonEquals(mPeriodicIO.pivot_motor_velocity, 0, 0.5)) {

    // setPivotPosition(mPeriodicIO.pivot_motor_position);
    // // System.out.println("triggered Pivot");
    // }
    // }

    public enum PivotControlState {
        OPEN_LOOP,
        CLOSED_LOOP
    }

    public void stop() {
        mIsEnabled = false;
        mPivot.set(0.0);
    }

    public double getPivotVelocity() {
        return mPeriodicIO.pivot_motor_velocity;
    }

    public double getPivotDemand() {
        return mPeriodicIO.pivot_demand;
    }

    /**
     * @return retruns the position of the pivot encoder
     */
    public double getPivotPosition() {
        return mPeriodicIO.pivot_cancoder;
    }

    /**
     * @return retruns stator current
     */
    public double getPivotCurrent() {
        return mPeriodicIO.pivot_current;
    }

    /**
     * @return retruns the PivotControlState
     */
    public PivotControlState getControlState() {
        return mPivotControlState;
    }

    public boolean checkSystem() {
        return true;
    }

    // public void setPivotDemand(double demand) {
    // mPeriodicIO.pivot_demand = demand;
    // }

    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(CTREConfigs.pivotCancoderConfig());
        angleEncoder.getPosition().setUpdateFrequency(300);
        angleEncoder.getVelocity().setUpdateFrequency(300);
        angleEncoder.getAbsolutePosition().setUpdateFrequency(300);
    }

    private void configPivotMotor(){
        mPivot.getConfigurator().apply(CTREConfigs.armPivotFXConfig());
        mPivot.set(0);
        mPivot.setInverted(false);
        mPivot.setRotorPosition(getCANCoderToMotorRotations());

    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue()*360);
    }

    public double getRelativeCancoder() {
        return angleEncoder.getPosition().getValue()*360 - Constants.PivotConstants.canCoderOffset;
    }

    public double getCANCoderToMotorRotations() {
        return getRelativeCancoder() * Constants.PivotConstants.oneDegreeOfroation;
    }

    public boolean hasEmergency = false;

    public void outputTelemetry() {
        SmartDashboard.putNumber("Pivot Demand", mPeriodicIO.pivot_demand);
        SmartDashboard.putNumber("Pivot Position", mPeriodicIO.pivot_motor_position);
        SmartDashboard.putNumber("CanCoder Position", getCANCoderToMotorRotations());
        SmartDashboard.putNumber("Pivot Voltage ", mPeriodicIO.pivot_voltage);
        SmartDashboard.putNumber("Pivot Current ", mPeriodicIO.pivot_current);
        SmartDashboard.putNumber("Pivot CanCoder Unadjusted", mPeriodicIO.pivot_cancoder);
        SmartDashboard.putNumber("Pivot CanCoder Adjusted", getRelativeCancoder());
        SmartDashboard.putNumber("Pivot Percent Duty Cycle", mPivot.getDutyCycle().getValue());

    }

    public static class PeriodicIO {
        /* Inputs */
        public double pivot_voltage;
        public double pivot_current;
        public double pivot_motor_position;
        public double pivot_motor_velocity;

        public double pivot_cancoder;

        /* Outputs */
        public double pivot_demand;
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "PIVOT_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");

        headers.add("is_enabled");

        headers.add("pivot_demand");

        headers.add("pivot_motor_position");

        headers.add("pivot_motor_velocity");

        headers.add("pivot_voltage");

        headers.add("pivot_stator_current");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());

        items.add(mIsEnabled ? 1.0 : 0.0);

        items.add(mPeriodicIO.pivot_demand);

        items.add(mPeriodicIO.pivot_motor_position);

        items.add(mPeriodicIO.pivot_motor_velocity);

        items.add(mPeriodicIO.pivot_voltage);

        items.add(mPeriodicIO.pivot_current);

        // send data to logging storage
        mStorage.addData(items);
    }
}
