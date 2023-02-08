package frc.robot.Subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team8013.frc2023.drivers.NeoMotor;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.logger.LogStorage;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team8013.frc2023.subsystems.ServoMotorSubsystem.ControlState;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;


//import edu.wpi.first.wpiutil.math.MathUtil;

public class Claw extends Subsystem {
    
    private static Claw mInstance;
    
    public static synchronized Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Claw();
        } 
        return mInstance;
    }
    
    // Pivot2 mPivot;
    double setPos;
    double curPos = 0;

    boolean isSnapping = false;

    CANSparkMax mPivotMotor;

    public ProfiledPIDController snapPIDController;

    private SparkMaxPIDController m_ClawPivotpidController;

    public RelativeEncoder m_PivEncoder;

    public Timer mTimer = new Timer();

    public Claw() {
        mPivotMotor = new CANSparkMax(Constants.gripPivID, MotorType.kBrushless);

        snapPIDController = new ProfiledPIDController(Constants.Claw.kP,
                Constants.Claw.kI,
                Constants.Claw.kD,
                Constants.Claw.kThetaControllerConstraints);
        snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController
         * object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_ClawPivotpidController = mPivotMotor.getPIDController();

        m_PivEncoder = mPivotMotor.getEncoder();

        m_PivEncoder.setPosition(0);

        /**
         * The PID Controller can be configured to use the analog sensor as its feedback
         * device with the method SetFeedbackDevice() and passing the PID Controller
         * the CANAnalog object.
         */
        m_ClawPivotpidController.setFeedbackDevice(m_PivEncoder);

        // mPivotMotor = new Pivot2(mPivotMotor, .2, 0.0, 1.0);

        // set PID coefficients
        m_ClawPivotpidController.setP(Constants.Claw.kP);
        m_ClawPivotpidController.setI(Constants.Claw.kI);
        m_ClawPivotpidController.setD(Constants.Claw.kD);
        m_ClawPivotpidController.setIZone(Constants.Claw.ClawIz);
        m_ClawPivotpidController.setFF(Constants.Claw.kFF);
        m_ClawPivotpidController.setOutputRange(Constants.Claw.kMinOutput, Constants.Claw.kMaxOutput);

    }

    // public void setRotation(int rotations) {
    // mPivotMotor.setPos(rotations);
    // }

    // public double calculateSnapValue() {
    // return snapPIDController.calculate(getRadianAngleOfClaw());
    // //mPigeon.getYaw().getRadians());
    // }

    // public double getRadianAngleOfClaw() {
    // return m_PivEncoder.getPosition() * 2 * Math.PI;
    // }

    // public void startClawSnap(double snapAngle) {
    // snapPIDController.reset(getRadianAngleOfClaw());
    // //mPigeon.getYaw().getRadians());
    // snapPIDController.setGoal(new
    // TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
    // isSnapping = true;
    // }

    // TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    // private boolean clawSnapComplete() {
    // double error = snapPIDController.getGoal().position - getRadianAngleOfClaw();
    // return delayedBoolean.update(Math.abs(error) <
    // Math.toRadians(Constants.Claw.kEpsilon),
    // Constants.Claw.kTimeout);
    // }

    // public void maybeStopClawSnap(boolean force) {
    // if (!isSnapping) {
    // return;
    // }
    // if (force || clawSnapComplete()) {
    // isSnapping = false;
    // snapPIDController.reset(getRadianAngleOfClaw());
    // }
    // }

    /*
     * recieves a position of degrees from 0 to 360
     * 
     * Out: Spins the claw pivot to the desired rotation
     */
    public void drivePivot(double desiredRotation) {

        double currentDegrees = getPivotPositionDegrees();

        // difference of desired rotation(0-360) to current position (0 to 1440)
        double difference = (currentDegrees % 360) - desiredRotation;

        double want = 1000;
        if (difference > 180) {
            if (currentDegrees + (360 - difference) <= 720) {
                want = (currentDegrees + (360 - difference));
            } else {
                want = (currentDegrees - difference);
            }

        } else if (difference > 0) {
            if (currentDegrees - difference >= -720) {
                want = (currentDegrees - difference);
            } else {
                want = (currentDegrees + (360 - difference));
            }

        } else if (difference < -180) {
            if (currentDegrees - (360 + difference) >= -720) { // -abs(difference)
                want = (currentDegrees - (360 + difference));
            } else {
                want = (currentDegrees - difference); // +abs(difference)
            }
        } else if (difference < 0) {
            if (currentDegrees - difference <= 720) {
                want = (currentDegrees - difference);
            } else {
                want = (currentDegrees - (360 + difference));
            }
        }
        if (want != 1000) {
            setPivotPosition(want / 360 * Constants.Claw.pivotGearRatio);
        }
    }

    // sets motor to specific number of rotations (dependent on current position;ex.
    // if pos=2, input=3, rotates once)

    private void setPivotPosition(double rotation) {
        SmartDashboard.putNumber("setPivPos", rotation / Constants.Claw.pivotGearRatio * 360);
        m_ClawPivotpidController.setReference(rotation, CANSparkMax.ControlType.kPosition);
    }

    // output: current position in degrees on a unit cirlce
    public double getPivotPositionDegrees() {
        return m_PivEncoder.getPosition() / Constants.Claw.pivotGearRatio * 360;
    }

    public void startSpin() {

    }

    public void stopSpinning() {

    }

    public void stopPositionIfClosing() {
        // System.out.println("thing:" + curPos);
        // double dt = Timer.getFPGATimestamp() - lastTime;
        // if (curPos + (m_speed * dt) < setPos) {
        // }
    }

    public void extendPosition() {
    }

    public void closePosition() {
    }

    public double getPosition() {
        return curPos;
    }

    public int getRotation() {
        return 2;
    }

    public void startTime() {
        mTimer.start();
    }

    public void resetClawSpin() {
        setPivotPosition(0);
    }

}
