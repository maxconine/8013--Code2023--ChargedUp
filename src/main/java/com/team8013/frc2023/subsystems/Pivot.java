package com.team8013.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.lib.util.CTREConfigs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.drivers.NeoMotor;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pivot extends Subsystem {

    private static Pivot mInstance;

    public static synchronized Pivot getInstance() {
        if (mInstance == null) {
            mInstance = new Pivot();
        }
        return mInstance;
    }

    private final CANSparkMax mPivot;
    private final NeoMotor mNeoMotor;

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    // status variable for is enabled
    public boolean mIsEnabled = false;

    public PivotControlState mPivotControlState = PivotControlState.OPEN_LOOP;

    private boolean mExtendPivot = false;
    private boolean mPartialExtendPivot = false;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private CANCoder angleEncoder;

    private Pivot() {
        mPivot = new CANSparkMax(Ports.PIVOT, MotorType.kBrushless);

        mNeoMotor = new NeoMotor(mPivot, Constants.PivotConstants.kP, Constants.PivotConstants.kI,
                Constants.PivotConstants.kD, Constants.PivotConstants.kMaxOutput, Constants.PivotConstants.kMinOutput);

        // Set defaults
        mPivot.set(0);
        mPivot.setInverted(true);
        mNeoMotor.setEncoderPosition(0.0);

        mPivot.setIdleMode(IdleMode.kBrake);

        // set current limits on motor
        mPivot.setSmartCurrentLimit(Constants.PivotConstants.kStatorCurrentLimit);

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

        mPeriodicIO.pivot_voltage = mPivot.getBusVoltage();
        mPeriodicIO.pivot_current = mPivot.getOutputCurrent();
        mPeriodicIO.pivot_motor_velocity = mNeoMotor.getMotorVelocityRPM();
        mPeriodicIO.pivot_motor_position = mNeoMotor.getEncoderPosition();

        // send log data
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        // set status variable for being enabled to true
        mIsEnabled = true;

        switch (mPivotControlState) {
            case OPEN_LOOP:
                mPivot.set(mPeriodicIO.pivot_demand / 12.0);
                break;
            case CLOSED_LOOP:
                mNeoMotor.setPosition(mPeriodicIO.pivot_demand);
                break;
            // case CLOSED_ENCODER: //use new pid controller that uses the encoder for input
            // mNeoMotor.s
            default:
                mNeoMotor.setPosition(0);
                break;
        }

    }

    public synchronized void setBrakeMode(boolean brake) {
        mPivot.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public synchronized void resetPivotPosition() {
        mNeoMotor.setEncoderPosition(0.0);
    }

    /*
     * @param wantedDemand
     * the dobule to set velocity to from 0-12
     */
    public void setPivotOpenLoop(double wantedDemand) {
        if (mPivotControlState != PivotControlState.OPEN_LOOP) {
            mPivotControlState = PivotControlState.OPEN_LOOP;
        }
        mPeriodicIO.pivot_demand = (wantedDemand > 12 ? 12 : wantedDemand);
    }

    public void setPivotPosition(double wantedPositionDegrees) {
        if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
            mPivotControlState = PivotControlState.CLOSED_LOOP;
        }
        mPeriodicIO.pivot_demand = wantedPositionDegrees * Constants.PivotConstants.oneDegreeOfroation;
    }

    // Sets the Pivot position to what its at + wantedPosition
    public void setPositionDelta(double wantedPositionDelta) {
        if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
            mPivotControlState = PivotControlState.CLOSED_LOOP;
            mPeriodicIO.pivot_demand = mPeriodicIO.pivot_motor_position;
        }
        mPeriodicIO.pivot_demand = mPeriodicIO.pivot_demand + wantedPositionDelta;
    }

    public void setPivotPosToCancoder() {
        mNeoMotor.setEncoderPosition(getRelativeCancoder() * Constants.PivotConstants.oneDegreeOfroation);
    }

    /***
     * Pivot METHODS
     * 
     *
     */

    public boolean canExtendArm() {
        return Math.abs(getPivotPosition()) > 20 * Constants.PivotConstants.oneDegreeOfroation;
    }

    public boolean canExtendArm(double degree) {
        return (Math.abs(getPivotPosition()) > ((degree - 1) * Constants.PivotConstants.oneDegreeOfroation))
                && (Math.abs(getPivotPosition()) < ((degree + 1) * Constants.PivotConstants.oneDegreeOfroation));
    }

    public void setPivotForPickup() {
        setPivotPosition(Constants.PivotConstants.kPickupTravelDistance);
    }

    // extend Pivot
    public void setPivotForHybrid() {
        setPivotPosition(Constants.PivotConstants.kHybridTravelDistance);
    }

    // third step for traversal
    public void setPivotForMid() {
        setPivotPosition(Constants.PivotConstants.kMidTravelDistance);
    }

    public void setPivotForHigh() {
        setPivotPosition(Constants.PivotConstants.kHighTravelDistance);
    }

    public void setPivotDown() {
        setPivotPosition(0); // rotations
    }

    // hold current position on Pivot
    // If Pivot is moving less than 0.5 m/s and Pivot current is greater than
    // constant
    // stator current limit
    public void maybeHoldCurrentPosition() {
        if (Util.epsilonEquals(mPeriodicIO.pivot_motor_velocity, 0, 0.5)) {

            setPivotPosition(mPeriodicIO.pivot_motor_position);
            // System.out.println("triggered Pivot");
        }
    }

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

    // returns rotations
    public double getPivotPosition() {
        return mPeriodicIO.pivot_motor_position;
    }

    /*
     * @return retruns current going to canSparkMax
     */
    public double getPivotCurrent() {
        return mPeriodicIO.pivot_current;
    }

    public PivotControlState getControlState() {
        return mPivotControlState;
    }

    public boolean checkSystem() {
        return true;
    }

    public void setPivotDemand(double demand) {
        mPeriodicIO.pivot_demand = demand;
    }

    public void setClimberDemand(double demand) {
        mPeriodicIO.pivot_demand = demand;
    }

    public void toggleExtendPivot() {
        mExtendPivot = !mExtendPivot;
    }

    public void togglePartialExtendPivot() {
        mPartialExtendPivot = !mPartialExtendPivot;
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.swerveCancoderConfig());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getRelativeCancoder() {
        return -1 * (angleEncoder.getPosition() - 138.25);
    }

    public boolean hasEmergency = false;

    public void outputTelemetry() {
        SmartDashboard.putNumber("Pivot Demand", mPeriodicIO.pivot_demand);
        SmartDashboard.putNumber("Pivot Position", mPeriodicIO.pivot_motor_position);
        SmartDashboard.putNumber("Pivot Voltage ", mPeriodicIO.pivot_voltage);
        SmartDashboard.putNumber("Pivot Current ", mPeriodicIO.pivot_current);

    }

    public static class PeriodicIO {
        /* Inputs */
        public double pivot_voltage;
        public double pivot_current;
        public double pivot_motor_position;
        public double pivot_motor_velocity;

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
