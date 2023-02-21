package com.team8013.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;
//import com.team8013.frc2023.subsystems.ServoMotorSubsystem.ControlState;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {

    private static Arm mInstance;

    public static synchronized Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    private final TalonFX mArm;

    public boolean mHomed;

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    // status variable for is enabled
    public boolean mIsEnabled = false;

    private boolean isPulledIn = false;

    public ArmControlState mArmControlState = ArmControlState.OPEN_LOOP;

    private boolean mExtendArm = false;
    private boolean mPartialExtendArm = false;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true,
            60, 60, .2);

    private Arm() {
        mArm = TalonFXFactory.createDefaultTalon(Ports.ARM);

        // Set defaults
        mArm.set(ControlMode.PercentOutput, 0);
        mArm.setInverted(true);
        mArm.setSelectedSensorPosition(0.0);

        mArm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mArm.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mArm.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mArm.config_kP(0, 0.6);
        mArm.config_kI(0, 0);
        mArm.config_kD(0, 0);
        mArm.config_kF(0, 0.077);

        mArm.setNeutralMode(NeutralMode.Brake);

        mArm.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mArm.enableVoltageCompensation(true);

        // set current limits on motor
        mArm.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
    }

    @Override
    public void readPeriodicInputs() {

        // curr check for holding
        maybeHoldCurrentPosition();

        mPeriodicIO.arm_voltage = mArm.getMotorOutputVoltage();
        mPeriodicIO.arm_stator_current = mArm.getStatorCurrent();
        mPeriodicIO.arm_motor_velocity = mArm.getSelectedSensorVelocity();
        mPeriodicIO.arm_motor_position = mArm.getSelectedSensorPosition();

        // send log data
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        // set status variable for being enabled to true
        mIsEnabled = true;

        switch (mArmControlState) {
            case OPEN_LOOP:
                mArm.set(ControlMode.PercentOutput, mPeriodicIO.arm_demand / 12.0);
                break;
            case MOTION_MAGIC:
                mArm.set(ControlMode.MotionMagic, mPeriodicIO.arm_demand);
                break;
            default:
                mArm.set(ControlMode.MotionMagic, 0.0);
                break;
        }

    }

    public synchronized void setBrakeMode(boolean brake) {
        mArm.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void resetClimberPosition() {
        mArm.setSelectedSensorPosition(0.0);
    }

    public void pullArmIntoZero() {
        // if (Math.abs(PeriodicIO.arm_motor_velocity) > 0.5)
        if (isPulledIn == false) {
            // SmartDashboard.putBoolean("isPulledIn", isPulledIn);
            setArmDemand(-10000);
            // pull in until velocity is less than 0.5 and current is higher than current
            // limit, reset position
            if (Util.epsilonEquals(mPeriodicIO.arm_motor_velocity, 0, 0.5)
                    && (mPeriodicIO.arm_stator_current > Constants.ArmConstants.kStatorCurrentLimit)) {
                resetClimberPosition();
                setArmPosition(10);
                isPulledIn = true;
                System.out.println("arm Pulled in");
            }
        }
    }

    public void setArmOpenLoop(double wantedDemand) {
        if (mArmControlState != ArmControlState.OPEN_LOOP) {
            mArmControlState = ArmControlState.OPEN_LOOP;
        }
        mPeriodicIO.arm_demand = (wantedDemand > 12 ? 12 : wantedDemand);
    }

    public void setArmPosition(double wantedPositionTicks) {
        if (mArmControlState != ArmControlState.MOTION_MAGIC) {
            mArmControlState = ArmControlState.MOTION_MAGIC;
        }
        mPeriodicIO.arm_demand = wantedPositionTicks;
    }

    public void setRightClimberPositionDelta(double wantedPositionDelta) {
        if (mArmControlState != ArmControlState.MOTION_MAGIC) {
            mArmControlState = ArmControlState.MOTION_MAGIC;
            mPeriodicIO.arm_demand = mPeriodicIO.arm_motor_position;
        }
        mPeriodicIO.arm_demand = mPeriodicIO.arm_demand + wantedPositionDelta;
    }

    /***
     * ARM METHODS
     * 
     *
     */

    // extend arm
    public void setExtendForHybrid() {
        isPulledIn = false;
        setArmPosition(Constants.ArmConstants.kHybridTravelDistance);
    }

    // third step for traversal
    public void setExtendForMid() {
        isPulledIn = false;
        setArmPosition(Constants.ArmConstants.kMidTravelDistance);
    }

    public void setExtendForHigh() {
        isPulledIn = false;
        setArmPosition(Constants.ArmConstants.kHighTravelDistance);
    }

    public void setArmDown() {
        setArmPosition(10); // ticks
    }

    // hold current position on arm
    // If arm is moving less than 0.5 m/s and arm current is greater than constant
    // stator current limit
    public void maybeHoldCurrentPosition() {
        if (Util.epsilonEquals(mPeriodicIO.arm_motor_velocity, 0, 0.5)
                && (mPeriodicIO.arm_stator_current > Constants.ArmConstants.kStatorCurrentLimit)) {

            setArmPosition(mPeriodicIO.arm_motor_position + 2000);
            System.out.println("triggered arm");
        }
    }

    public enum ArmControlState {
        HOMING,
        OPEN_LOOP,
        MOTION_MAGIC
    }

    public void stop() {
        mIsEnabled = false;
        mArm.set(ControlMode.PercentOutput, 0.0);
    }

    public double getArmVelocity() {
        return mPeriodicIO.arm_motor_velocity;
    }

    public double getArmDemand() {
        return mPeriodicIO.arm_demand;
    }

    public double getArmPosition() {
        return mPeriodicIO.arm_motor_position;
    }

    public double getArmCurrent() {
        return mPeriodicIO.arm_stator_current;
    }

    public boolean getHomed() {
        return mHomed;
    }

    public ArmControlState getControlState() {
        return mArmControlState;
    }

    public boolean checkSystem() {
        return true;
    }

    public void setArmDemand(double demand) {
        mPeriodicIO.arm_demand = demand;
    }

    public void setClimberDemand(double demand) {
        mPeriodicIO.arm_demand = demand;
    }

    public void toggleExtendArm() {
        mExtendArm = !mExtendArm;
    }

    public void togglePartialExtendArm() {
        mPartialExtendArm = !mPartialExtendArm;
    }

    public boolean getExtendArm() {
        return mExtendArm;
    }

    public boolean getPartialExtendArm() {
        return mPartialExtendArm;
    }

    public boolean hasEmergency = false;

    public void outputTelemetry() {
        SmartDashboard.putNumber("Arm Demand", mPeriodicIO.arm_demand);
        SmartDashboard.putNumber("Arm Position", mPeriodicIO.arm_motor_position);
        SmartDashboard.putNumber("Arm Voltage ", mPeriodicIO.arm_voltage);
        SmartDashboard.putNumber("Arm Current ", mPeriodicIO.arm_stator_current);
        SmartDashboard.putBoolean("Extend Arm", getExtendArm());
        SmartDashboard.putBoolean("Partial Extend Arm", getPartialExtendArm());

    }

    public static class PeriodicIO {
        /* Inputs */
        public double arm_voltage;
        public double arm_stator_current;
        public double arm_motor_position;
        public double arm_motor_velocity;

        /* Outputs */
        public double arm_demand;
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "ARM_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");

        headers.add("is_enabled");

        headers.add("arm_demand");

        headers.add("arm_motor_position");

        headers.add("arm_motor_velocity");

        headers.add("arm_voltage");

        headers.add("arm_stator_current");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());

        items.add(mIsEnabled ? 1.0 : 0.0);

        items.add(mPeriodicIO.arm_demand);

        items.add(mPeriodicIO.arm_motor_position);

        items.add(mPeriodicIO.arm_motor_velocity);

        items.add(mPeriodicIO.arm_voltage);

        items.add(mPeriodicIO.arm_stator_current);

        // send data to logging storage
        mStorage.addData(items);
    }
}
