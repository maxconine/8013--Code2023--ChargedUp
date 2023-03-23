package com.team8013.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
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

public class PivotV3 extends Subsystem {

    private static PivotV3 mInstance;

    public static synchronized PivotV3 getInstance() {
        if (mInstance == null) {
            mInstance = new PivotV3();
        }
        return mInstance;
    }

    private final TalonFX mPivot;
    //private final PIDController mPivotPid;

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    // status variable for is enabled
    public boolean mIsEnabled = false;

    public PivotControlState mPivotControlState = PivotControlState.OPEN_LOOP;

    // private boolean mExtendPivot = false;
    // private boolean mPartialExtendPivot = false;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private CANCoder angleEncoder;


    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true,
            Constants.PivotConstants.kStatorCurrentLimit, 60, .2);

    private PivotV3() {
        mPivot = TalonFXFactory.createDefaultTalon(Ports.PIVOT); //TODO:SET THE PORT

        // Set defaults
        mPivot.set(ControlMode.PercentOutput, 0);
        mPivot.setInverted(false); //TODO: is this inverted?
        mPivot.setSelectedSensorPosition(getCANCoderToMotorTicks());

        mPivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mPivot.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mPivot.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mPivot.config_kP(0, 0.3);
        mPivot.config_kI(0, 0);
        mPivot.config_kD(0, 0);
        mPivot.config_kF(0, 0.0); //Is this supposed to be 0?


        // mPivotPid = new PIDController(Constants.PivotConstants.kP, Constants.PivotConstants.kI,
        // Constants.PivotConstants.kD);
        // mPivotPid.disableContinuousInput();

        mPivot.setNeutralMode(NeutralMode.Brake);

        mPivot.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mPivot.enableVoltageCompensation(true);

        // set current limits on motor
        mPivot.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);

        angleEncoder = new CANCoder(Ports.PIV_CANCODER, "canivore1");
        configAngleEncoder();
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 300); // originally 255
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 300); // originally 255
    }

    @Override
    public void readPeriodicInputs() {

        // curr check for holding'
        // TODO: i commented this out
        // maybeHoldCurrentPosition();

        mPeriodicIO.pivot_voltage = mPivot.getMotorOutputVoltage();
        mPeriodicIO.pivot_current = mPivot.getStatorCurrent();
        mPeriodicIO.pivot_motor_velocity = mPivot.getSelectedSensorVelocity();
        mPeriodicIO.pivot_motor_position_ticks = mPivot.getSelectedSensorPosition();

        mPeriodicIO.pivot_cancoder = getRelativeCancoder();

        // send log data
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        // set status variable for being enabled to true
        mIsEnabled = true;

        if (Math.abs(getCANCoderToMotorTicks() - mPeriodicIO.pivot_motor_position_ticks)<Constants.PivotConstants.ticksOffToReset){
            mPivot.setSelectedSensorPosition(getCANCoderToMotorTicks());
        }
        double feedForward = Constants.PivotConstants.kGravity * Math.cos(getRelativeCancoder()-90); //0 degrees is horisontal for this

        switch (mPivotControlState) {
            case OPEN_LOOP:
                mPivot.set(ControlMode.PercentOutput, mPeriodicIO.pivot_demand, DemandType.ArbitraryFeedForward, feedForward);
                break;
            case CLOSED_LOOP:
            mPivot.set(ControlMode.MotionMagic, mPeriodicIO.pivot_demand, DemandType.ArbitraryFeedForward, feedForward);

            // mPivotPid.setSetpoint(mPeriodicIO.pivot_demand);
            // mPivot.set(ControlMode.PercentOutput,
            //         MathUtil.clamp(mPivotPid.calculate(mPeriodicIO.pivot_cancoder),
            //                 Constants.PivotConstants.kMinOutput,
            //                 Constants.PivotConstants.kMaxOutput));

                break;
            // case CLOSED_ENCODER: //use new pid controller that uses the encoder for input
            // mNeoMotor.s
            default:
                mPivot.set(ControlMode.PercentOutput, 0);
                break;
        }

    }

    public synchronized void setBrakeMode(boolean brake) {
        mPivot.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void resetPivotPosition() {
        mPivot.setSelectedSensorPosition(0.0);
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
        mPeriodicIO.pivot_demand = wantedPositionDegrees * Constants.PivotConstants.oneDegreeOfroation * 2048.0;
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
        mPeriodicIO.pivot_demand = mPeriodicIO.pivot_demand + (wantedPositionDelta * Constants.PivotConstants.oneDegreeOfroation * 2048.0);
    }

    //Basically does nothing
    // public void setPivotPosToCancoder() {
    //     if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
    //         mPivotControlState = PivotControlState.CLOSED_LOOP;
    //     }
    //     mPeriodicIO.pivot_demand = getRelativeCancoder();
    // }

    /*Pivot METHODS*/

    /**
     * 
     * @return true if the pivot is rotated 30 degrees
     */
    public boolean canExtendArm() {
        return Math.abs(mPeriodicIO.pivot_cancoder) > 30;
    }

        /**
     * @param degree the degree that you want the pivot to turn to
     * @return true if the pivot is within 4 degrees of that degree you want it to turn
     */
    public boolean canExtendArm(double degree) {
        return (Math.abs((Math.abs(mPeriodicIO.pivot_cancoder) - Math.abs(degree))) < Constants.PivotConstants.degreesCanExtendArm);
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
    //     if (Util.epsilonEquals(mPeriodicIO.pivot_motor_velocity, 0, 0.5)) {

    //         setPivotPosition(mPeriodicIO.pivot_motor_position);
    //         // System.out.println("triggered Pivot");
    //     }
    // }

    public enum PivotControlState {
        OPEN_LOOP,
        CLOSED_LOOP
    }

    public void stop() {
        mIsEnabled = false;
        mPivot.set(ControlMode.PercentOutput, 0.0);
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
    //     mPeriodicIO.pivot_demand = demand;
    // }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.pivotCancoderConfig());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getRelativeCancoder() {
        return angleEncoder.getPosition() - Constants.PivotConstants.canCoderOffset;
    }

    public double getCANCoderToMotorTicks(){
        return getRelativeCancoder() * Constants.PivotConstants.oneDegreeOfroation * 2048;
    }

    public boolean hasEmergency = false;

    public void outputTelemetry() {
        SmartDashboard.putNumber("Pivot Demand", mPeriodicIO.pivot_demand);
        SmartDashboard.putNumber("Pivot Position Ticks", mPeriodicIO.pivot_motor_position_ticks);
        SmartDashboard.putNumber("CanCoder Position Ticks", getCANCoderToMotorTicks());
        SmartDashboard.putNumber("Pivot Voltage ", mPeriodicIO.pivot_voltage);
        SmartDashboard.putNumber("Pivot Current ", mPeriodicIO.pivot_current);
        SmartDashboard.putNumber("Pivot CanCoder Unadjusted", mPeriodicIO.pivot_cancoder);
        SmartDashboard.putNumber("Pivot CanCoder Adjusted", getRelativeCancoder());
        SmartDashboard.putNumber("Pivot Percent Output", mPivot.getMotorOutputPercent());
    }

    public static class PeriodicIO {
        /* Inputs */
        public double pivot_voltage;
        public double pivot_current;
        public double pivot_motor_position_ticks;
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

        items.add(mPeriodicIO.pivot_motor_position_ticks);

        items.add(mPeriodicIO.pivot_motor_velocity);

        items.add(mPeriodicIO.pivot_voltage);

        items.add(mPeriodicIO.pivot_current);

        // send data to logging storage
        mStorage.addData(items);
    }
}

