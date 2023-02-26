package com.team8013.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawV2 extends Subsystem {

    private static ClawV2 mInstance;

    public static synchronized ClawV2 getInstance() {
        if (mInstance == null) {
            mInstance = new ClawV2();
        }
        return mInstance;
    }

    PIDController m_PivotPid;
    VictorSPX m_PivotMotor;
    Encoder m_PivotEncoder;

    PIDController m_GripPid;
    VictorSPX m_GripMotor;
    Encoder m_GripEncoder;

    double doubleCheckBrokenTime = 0;

    public GripControlState mGripControlState = GripControlState.OPEN_LOOP;
    public PivotControlState mPivotControlState = PivotControlState.OPEN_LOOP;

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    // status variable for is enabled
    public boolean mIsEnabled = false;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private ClawV2() {
        m_PivotEncoder = new Encoder(Ports.CLAW_PIV_ENCODER_A, Ports.CLAW_PIV_ENCODER_B);
        m_PivotEncoder.setDistancePerPulse(1 / 44.4); // / Constants.ClawConstants.pivotGearRatio);
        m_PivotMotor = new VictorSPX(Ports.CLAW_PIV);

        m_GripEncoder = new Encoder(Ports.CLAW_GRIP_ENCODER_A, Ports.CLAW_GRIP_ENCODER_B);
        m_GripEncoder.setDistancePerPulse(1 / 44.4);
        m_GripMotor = new VictorSPX(Ports.CLAW_GRIP);

        m_GripEncoder.setReverseDirection(true); // in is positive, out is negative
        m_PivotEncoder.setReverseDirection(false); // CCW+ in is positive

        m_PivotPid = new PIDController(Constants.ClawConstants.piv_kP, Constants.ClawConstants.piv_kI,
                Constants.ClawConstants.piv_kD);
        m_GripPid = new PIDController(Constants.ClawConstants.grip_kP, Constants.ClawConstants.grip_kI,
                Constants.ClawConstants.grip_kD);

        mPeriodicIO.wantedClosing = false;
        mPeriodicIO.encoderBroken = false;
        mPeriodicIO.maybeEncoderBroken = false;
    }

    @Override
    public void readPeriodicInputs() {

        /* PIVOT MOTOR */
        mPeriodicIO.pivot_voltage = m_PivotMotor.getBusVoltage();
        mPeriodicIO.pivot_current = m_PivotMotor.getMotorOutputVoltage();
        mPeriodicIO.pivot_motor_velocity = m_PivotEncoder.getRate();
        mPeriodicIO.pivot_motor_position = m_PivotEncoder.getDistance();

        /* CLAW MOTOR */
        mPeriodicIO.grip_voltage = m_GripMotor.getBusVoltage();
        mPeriodicIO.grip_current = m_GripMotor.getMotorOutputVoltage();
        mPeriodicIO.grip_motor_velocity = m_GripEncoder.getRate();
        mPeriodicIO.grip_motor_position = m_GripEncoder.getDistance();

        // mPeriodicIO.wantedClosing =

        // send log data
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        // set status variable for being enabled to true
        mIsEnabled = true;

        // if (((mPeriodicIO.grip_current > 11) && (mPeriodicIO.grip_motor_position ==
        // 0.0))
        // || (mPeriodicIO.pivot_current > 11) && (mPeriodicIO.pivot_motor_position ==
        // 0.0)
        // && (!mPeriodicIO.maybeEncoderBroken)) {
        // doubleCheckBrokenTime = Timer.getFPGATimestamp();
        // mPeriodicIO.maybeEncoderBroken = true;
        // }
        // if ((Timer.getFPGATimestamp() + 0.5 > doubleCheckBrokenTime)
        // && (((mPeriodicIO.grip_current > 11) && (mPeriodicIO.grip_motor_position ==
        // 0.0))
        // || (mPeriodicIO.pivot_current > 11) && (mPeriodicIO.pivot_motor_position ==
        // 0.0))
        // && (mPeriodicIO.maybeEncoderBroken)) {
        // mPeriodicIO.encoderBroken = true;
        // }

        // if (mPeriodicIO.encoderBroken == false) {

        /* GRIP OUTPUT */

        switch (mGripControlState) {
            case OPEN_LOOP:
                // if trying to close:
                if (mPeriodicIO.wantedClosing == true) {
                    m_GripMotor.set(VictorSPXControlMode.PercentOutput, -Constants.ClawConstants.grip_kMaxOutput);

                    mPeriodicIO.grip_peakSpeed = Math.max(mPeriodicIO.grip_motor_velocity,
                            mPeriodicIO.grip_peakSpeed);

                    // if grip rate slows down to ex: 70% of max speed greater than 2, or claw is
                    // too far in
                    if ((mPeriodicIO.grip_motor_velocity > 2)
                            && (mPeriodicIO.grip_motor_velocity < (mPeriodicIO.grip_peakSpeed
                                    * Constants.ClawConstants.grip_rateDiff))
                            || (mPeriodicIO.grip_motor_position <= Constants.ClawConstants.kClawMinDistance)) {
                        m_GripMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
                        mPeriodicIO.wantedClosing = false;
                        mPeriodicIO.grip_peakSpeed = 0;
                        // mPeriodicIO.spedUp = false;
                    }
                }
                // if trying to open manually, dont let the claw over extend out:
                else if ((mPeriodicIO.grip_demand < 0) // I changed signs around
                        && (mPeriodicIO.grip_motor_position < Constants.ClawConstants.kClawMaxDistance)) {
                    m_GripMotor.set(VictorSPXControlMode.PercentOutput, mPeriodicIO.grip_demand);
                } else if ((mPeriodicIO.grip_demand > 0)
                        && (mPeriodicIO.grip_motor_position > Constants.ClawConstants.kClawMinDistance)) {
                    m_GripMotor.set(VictorSPXControlMode.PercentOutput, mPeriodicIO.grip_demand);
                } else {
                    m_GripMotor.set(VictorSPXControlMode.PercentOutput, 0);
                }
                break;
            case CLOSED_LOOP:
                // don't let distance exceed max and min
                mPeriodicIO.grip_demand = MathUtil.clamp(mPeriodicIO.grip_demand,
                        Constants.ClawConstants.kClawMinDistance, Constants.ClawConstants.kClawMaxDistance);
                m_GripPid.setSetpoint(mPeriodicIO.grip_demand);
                m_GripMotor.set(VictorSPXControlMode.PercentOutput,
                        MathUtil.clamp(/*-1 */ m_GripPid.calculate(mPeriodicIO.grip_motor_position),
                                Constants.ClawConstants.grip_kMinOutput,
                                Constants.ClawConstants.grip_kMaxOutput));

                break;
            default:
                m_GripMotor.set(VictorSPXControlMode.PercentOutput, 0);
                break;

        }

        /* PIVOT OUTPUT */
        switch (mPivotControlState) {
            case OPEN_LOOP:
                // if ((mPeriodicIO.pivot_motor_position >
                // Constants.ClawConstants.kPivotMaxDistance)
                // && (mPeriodicIO.pivot_demand > 0)) {
                // m_PivotMotor.set(VictorSPXControlMode.PercentOutput, 0);
                // } else if ((mPeriodicIO.pivot_motor_position <
                // Constants.ClawConstants.kPivotMinDistance)
                // && (mPeriodicIO.pivot_demand < 0)) {
                // m_PivotMotor.set(VictorSPXControlMode.PercentOutput, 0);
                // } else {
                m_PivotMotor.set(VictorSPXControlMode.PercentOutput, mPeriodicIO.pivot_demand);
                // }
                break;
            case CLOSED_LOOP:
                mPeriodicIO.pivot_demand = MathUtil.clamp(mPeriodicIO.pivot_demand,
                        Constants.ClawConstants.kPivotMinDistance, Constants.ClawConstants.kPivotMaxDistance);
                m_PivotPid.setSetpoint(mPeriodicIO.pivot_demand);
                m_PivotMotor.set(VictorSPXControlMode.PercentOutput,
                        MathUtil.clamp(-1 * m_PivotPid.calculate(mPeriodicIO.pivot_motor_position),
                                Constants.ClawConstants.piv_kMinOutput,
                                Constants.ClawConstants.piv_kMaxOutput));
                break;
            default:
                m_PivotMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
                break;

        }

    }

    public synchronized void resetPositions() {
        m_GripEncoder.reset();
        m_PivotEncoder.reset();
    }

    public void resetPivotEncoder() {
        m_PivotEncoder.reset();
    }

    public void resetGripEncoder() {
        m_GripEncoder.reset();
    }

    /**
     * @param wantedDemand
     *                     the dobule to set velocity to from -1 to 1
     */
    public void setPivotOpenLoop(double wantedDemand) {
        if (mPivotControlState != PivotControlState.OPEN_LOOP) {
            mPivotControlState = PivotControlState.OPEN_LOOP;
        }
        mPeriodicIO.pivot_demand = MathUtil.clamp(wantedDemand, -1, 1);
    }

    /**
     * @param wantedDemand
     *                     the dobule to set velocity to from -1 to 1
     */
    public void setGripOpenLoop(double wantedDemand) {
        if (mGripControlState != GripControlState.OPEN_LOOP) {
            mGripControlState = GripControlState.OPEN_LOOP;
        }
        mPeriodicIO.grip_demand = MathUtil.clamp(wantedDemand, -1, 1);
    }

    public void setPivotPosition(double wantedPositionDegrees) {
        if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
            mPivotControlState = PivotControlState.CLOSED_LOOP;
        }
        // demand in rotations
        mPeriodicIO.pivot_demand = wantedPositionDegrees / 180;
    }

    public void setGripPosition(double wantedPositionRotations) {
        if (mPivotControlState != PivotControlState.CLOSED_LOOP) {
            mPivotControlState = PivotControlState.CLOSED_LOOP;
        }
        // demand in rotations
        mPeriodicIO.pivot_demand = wantedPositionRotations;
    }

    public void setClawForPickup() {
        setPivotPosition(0);
        openGrip();
    }

    public void openGrip() {
        if (mGripControlState != GripControlState.CLOSED_LOOP) {
            mGripControlState = GripControlState.CLOSED_LOOP;
        }
        mPeriodicIO.grip_demand = Constants.ClawConstants.kClawOpenDistance;
    }

    public void closeGrip() {
        if (mGripControlState != GripControlState.OPEN_LOOP) {
            mGripControlState = GripControlState.OPEN_LOOP;
        }
        mPeriodicIO.wantedClosing = true;
        mPeriodicIO.grip_demand = 0;
        // mPeriodicIO.grip_demand = Constants.ClawConstants.grip_kMaxOutput;
    }

    // input: desired rotation(0-360)
    public void drivePivot(double desiredRotation) {

        double currentDegrees = m_PivotEncoder.getDistance() * 360;

        // difference of desired rotation(0-360) to current position (0 to 1440)
        double difference = (currentDegrees % 360) - desiredRotation;

        double want = 1000;
        if (difference > 180) {
            if (currentDegrees + (360 - difference) <= Constants.ClawConstants.piv_MaxRotation) {
                want = (currentDegrees + (360 - difference));
            } else {
                want = (currentDegrees - difference);
            }

        } else if (difference > 0) {
            if (currentDegrees - difference >= Constants.ClawConstants.piv_MinRotation) {
                want = (currentDegrees - difference);
            } else {
                want = (currentDegrees + (360 - difference));
            }

        } else if (difference < -180) {
            if (currentDegrees - (360 + difference) >= Constants.ClawConstants.piv_MinRotation) { // -abs(difference)
                want = (currentDegrees - (360 + difference));
            } else {
                want = (currentDegrees - difference); // +abs(difference)
            }
        } else if (difference < 0) {
            if (currentDegrees - difference <= Constants.ClawConstants.piv_MaxRotation) {
                want = (currentDegrees - difference);
            } else {
                want = (currentDegrees - (360 + difference));
            }
        }
        if (want != 1000) {
            m_PivotPid.setSetpoint(want / 360); // should just put this where ever the value
                                                // of want is being changed
        }
    }

    public enum GripControlState {
        OPEN_LOOP,
        CLOSED_LOOP
    }

    public enum PivotControlState {
        OPEN_LOOP,
        CLOSED_LOOP
    }

    public void stop() {
        mIsEnabled = false;
        m_GripMotor.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.grip_demand = 0;
        m_PivotMotor.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.pivot_demand = 0;
        mPeriodicIO.wantedClosing = false;
    }

    public void stopGrip() {
        mPeriodicIO.grip_demand = 0;
    }

    public void stopPivot() {
        mPeriodicIO.pivot_demand = 0;
    }

    public double getPivotVelocity() {
        return mPeriodicIO.pivot_motor_velocity;
    }

    // returns percent output
    public double getPivotDemand() {
        return mPeriodicIO.pivot_demand;
    }

    // returns rotations of motor
    public double getPivotPosition() {
        return mPeriodicIO.pivot_motor_position;
    }

    /*
     * @return retruns current going to canSparkMax
     */
    public double getPivotCurrent() {
        return mPeriodicIO.grip_current;
    }

    public double getGripVelocity() {
        return mPeriodicIO.grip_motor_velocity;
    }

    // returns percent output
    public double getGripDemand() {
        return mPeriodicIO.grip_demand;
    }

    // returns rotations of motor
    public double getGripPosition() {
        return mPeriodicIO.grip_motor_position;
    }

    // @return retruns current going to canSparkMax
    public double getGripCurrent() {
        return mPeriodicIO.grip_current;
    }

    public boolean checkSystem() {
        return true;
    }

    public void setPivotDemand(double demand) {
        mPeriodicIO.pivot_demand = demand;
    }

    public void setGripDemand(double demand) {
        mPeriodicIO.grip_demand = demand;
    }

    public boolean hasEmergency = false;

    public void outputTelemetry() {
        SmartDashboard.putNumber("Claw Pivot Demand", mPeriodicIO.pivot_demand);
        SmartDashboard.putNumber("Claw Pivot Position", mPeriodicIO.pivot_motor_position);
        SmartDashboard.putNumber("Claw Pivot Voltage ", mPeriodicIO.pivot_voltage);
        SmartDashboard.putNumber("Claw Pivot Current ", mPeriodicIO.pivot_current);

        SmartDashboard.putNumber("Claw Grip Demand", mPeriodicIO.grip_demand);
        SmartDashboard.putNumber("Claw Grip Position", mPeriodicIO.grip_motor_position);
        SmartDashboard.putNumber("Claw Grip Voltage ", mPeriodicIO.grip_voltage);
        SmartDashboard.putNumber("Claw Grip Current ", mPeriodicIO.grip_current);

        SmartDashboard.putBoolean("Claw want grip", mPeriodicIO.wantedClosing);
        SmartDashboard.putNumber("Claw peak speed", mPeriodicIO.grip_peakSpeed);

        SmartDashboard.putBoolean("Claw Encoders Broken", mPeriodicIO.encoderBroken);
        SmartDashboard.putBoolean("Claw Encoders MAYBE Broken", mPeriodicIO.maybeEncoderBroken);
    }

    public static class PeriodicIO {
        /* Inputs */
        public double pivot_voltage;
        public double pivot_current;
        public double pivot_motor_position;
        public double pivot_motor_velocity;

        public double grip_voltage;
        public double grip_current;
        public double grip_motor_position;
        public double grip_motor_velocity;

        public boolean encoderBroken;
        public boolean maybeEncoderBroken;

        /* Outputs */
        public double pivot_demand; // position
        public double grip_demand; // either percent output or position
        public double grip_peakSpeed;

        public boolean wantedClosing;
        // public boolean spedUp;
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "CLAW_LOGS.csv");
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

        headers.add("grip_demand");

        headers.add("grip_motor_position");

        headers.add("grip_motor_velocity");

        headers.add("grip_voltage");

        headers.add("grip_stator_current");

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

        items.add(mPeriodicIO.grip_motor_position);

        items.add(mPeriodicIO.grip_motor_velocity);

        items.add(mPeriodicIO.grip_voltage);

        items.add(mPeriodicIO.grip_current);

        // send data to logging storage
        mStorage.addData(items);
    }
}
