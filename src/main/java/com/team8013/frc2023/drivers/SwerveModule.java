package com.team8013.frc2023.drivers;

import com.lib.math.Conversions;
import com.lib.util.CTREConfigs;
import com.lib.util.CTREModuleState;
import com.lib.util.SwerveModuleConstants;
import com.team254.lib.drivers.TalonFXFactory;
import com.team8013.frc2023.Constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.hardware.CANcoder;

public class SwerveModule {
    public int moduleNumber;
    public double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private double lastAngle;
    private DutyCycleOut driveDutyCycleOut;
    private VelocityTorqueCurrentFOC driveVelocityTorqueCurrentFOC;
    PositionTorqueCurrentFOC anglePositionControl;
    PositionDutyCycle anglePositionDutyCycle;

    private double anglekP;
    private double anglekI;
    private double anglekD;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS,
            Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        // TODO:
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "canivore1");
        configAngleEncoder();
        //angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 300); // originally 255
        //angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 300); // originally 255
        angleEncoder.getPosition().setUpdateFrequency(300);
        angleEncoder.getVelocity().setUpdateFrequency(300);
        angleEncoder.getAbsolutePosition().setUpdateFrequency(300);

        /* Angle Motor Config */
        mAngleMotor = TalonFXFactory.createDefaultTalon(moduleConstants.angleMotorID);
        configAngleMotor();
        TalonFXConfiguration angleConfiguration = CTREConfigs.swerveAngleFXConfig();
        anglekP = angleConfiguration.Slot0.kP;
        anglekI = angleConfiguration.Slot0.kI;
        anglekD = angleConfiguration.Slot0.kD;

        /* Drive Motor Config */
        mDriveMotor = TalonFXFactory.createDefaultTalon(moduleConstants.driveMotorID);
        configDriveMotor();
        mDriveMotor.setRotorPosition(0, Constants.kLongCANTimeoutMs);

        // configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        //         Constants.kLongCANTimeoutMs);
        driveDutyCycleOut = new DutyCycleOut(0);
        driveDutyCycleOut.OverrideBrakeDurNeutral = true;
        driveDutyCycleOut.EnableFOC = true;

        driveVelocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        driveVelocityTorqueCurrentFOC.OverrideCoastDurNeutral = true;

        anglePositionControl = new PositionTorqueCurrentFOC(0);
        anglePositionControl.Slot = 0;

        //TODO: DOES THIS FOC CONTROL WORK OR THIS DUTY CYCLE CONTROL WORK?

        anglePositionDutyCycle = new PositionDutyCycle(0);
        anglePositionDutyCycle.Slot = 0;

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveDutyCycleOut.Output = percentOutput;
            mDriveMotor.setControl(new DutyCycleOut(percentOutput));

        } else {
            double velocity = Conversions.MPSToMotorVelocity(desiredState.speedMetersPerSecond,
                     Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);

            driveVelocityTorqueCurrentFOC.Velocity = velocity;
            driveVelocityTorqueCurrentFOC.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);

            mDriveMotor.setControl(driveVelocityTorqueCurrentFOC);

        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        // mAngleMotor.set(ControlMode.Position,
        //         Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));

        anglePositionControl.Position = Conversions.degreesToMotorRoation(angle, Constants.SwerveConstants.angleGearRatio);
        
        mAngleMotor.setControl(anglePositionControl);

        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToMotorRoation(getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        // angleEncoder.getConfigurator().
        angleEncoder.getConfigurator().apply(CTREConfigs.swerveCancoderConfig());
    }

    private void configAngleMotor() {
        // mAngleMotor.configFactoryDefault();
        mAngleMotor.getConfigurator().apply(CTREConfigs.swerveAngleFXConfig());
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);

        //mAngleMotor.setControl(Constants.SwerveConstants.angleNeutralMode); // setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        // mDriveMotor.configFactoryDefault();
        mDriveMotor.getConfigurator().apply(CTREConfigs.swerveDriveFXConfig());
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        // mDriveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        mDriveMotor.setRotorPosition(0);
    }

    public void updateAnglePID(double kP, double kI, double kD) {
        if (anglekP != kP) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = kP;
            anglekP = kP;
            mAngleMotor.getConfigurator().apply(slot0Configs, Constants.kLongCANTimeoutMs);// .config_kP(0, anglekP, Constants.kLongCANTimeoutMs);
        }
        if (anglekI != kI) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kI = kI;
            anglekI = kI;
            mAngleMotor.getConfigurator().apply(slot0Configs, Constants.kLongCANTimeoutMs);
        }
        if (anglekD != kD) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kD = kD;
            anglekD = kD;
            mAngleMotor.getConfigurator().apply(slot0Configs, Constants.kLongCANTimeoutMs);
        }
    }

    public double[] getAnglePIDValues() {
        double[] values = { anglekP, anglekI, anglekD };
        return values;
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue()*360);
    }

    public double getTargetAngle() {
        return lastAngle;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getRotorVelocity().getValue(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getRotorPosition().getValue(),
                Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(Conversions
                .falconToDegrees(mAngleMotor.getRotorPosition().getValue(), Constants.SwerveConstants.angleGearRatio)));
        // Potential Breaking Point^
    }

    public double getDrivePosition() {
        return Conversions.falconToMeters(mDriveMotor.getRotorPosition().getValue(),
            Constants.SwerveConstants.driveGearRatio, Constants.SwerveConstants.wheelCircumference);
                //  * 0.58577049438;
    }

    // public double getDriveDistance() {
    // return getStateDistance();
    // }

    // default SwerveModulePosition getPosition() {
    // return new SwerveModulePosition(getDriveDistance(),
    // Rotation2d.fromRadians(angleEncoder.getPosition()));
    // }

}
