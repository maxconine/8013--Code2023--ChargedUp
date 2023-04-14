package com.lib.util;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MagnetSensorConfigs;
import com.ctre.phoenixpro.configs.OpenLoopRampsConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team8013.frc2023.Constants;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();

        driveSupplyLimit.StatorCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
        driveSupplyLimit.StatorCurrentLimit = Constants.SwerveConstants.StatorCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = Constants.SwerveConstants.driveSupplyCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveSupplyCurrentLimitEnable;

        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
                // Constants.SwerveConstants.driveEnableCurrentLimit,
                // Constants.SwerveConstants.driveContinuousCurrentLimit,
                // Constants.SwerveConstants.drivePeakCurrentLimit,
                // Constants.SwerveConstants.drivePeakCurrentDuration);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = Constants.SwerveConstants.driveKP;
        config.Slot0.kI = Constants.SwerveConstants.driveKI;
        config.Slot0.kD = Constants.SwerveConstants.driveKD;
        config.Slot0.kS = Constants.SwerveConstants.driveKS;
        config.Slot0.kV = Constants.SwerveConstants.driveKV;
        config.CurrentLimits = driveSupplyLimit;
        // config. = SensorInitializationStrategy.BootToZero;
        config.OpenLoopRamps = openLoopRampsConfigs;
        config.ClosedLoopRamps = closedLoopRampsConfigs;

        return config;
    }

    public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        ClosedLoopRampsConfigs angleClosedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs angleOpenLoopRampsConfigs = new OpenLoopRampsConfigs();

        angleSupplyLimit.StatorCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
        angleSupplyLimit.StatorCurrentLimit = Constants.SwerveConstants.angleStatorCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = Constants.SwerveConstants.angleSupplyCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleSupplyCurrentLimitEnable;

        angleClosedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        angleOpenLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        angleConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
        angleConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
        angleConfig.Slot0.kD = Constants.SwerveConstants.angleKD;

        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // angleConfig.Slot0.kS = Constants.SwerveConstants.angleKS;
        // angleConfig.Slot0.kV = Constants.SwerveConstants.angleKV;
        angleConfig.CurrentLimits = angleSupplyLimit;
        angleConfig.OpenLoopRamps = angleOpenLoopRampsConfigs;
        angleConfig.ClosedLoopRamps = angleClosedLoopRampsConfigs;
        // angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return angleConfig;
    }

    public static CANcoderConfiguration swerveCancoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs swerveCANcoderConfigs = new MagnetSensorConfigs();
        swerveCANcoderConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCANcoderConfigs.SensorDirection = Constants.SwerveConstants.canCoderInvert;

        config.MagnetSensor = swerveCANcoderConfigs;
        return config;
    }

    public static CANcoderConfiguration pivotCancoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs pivotCANcoderConfigs = new MagnetSensorConfigs();
        pivotCANcoderConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        pivotCANcoderConfigs.SensorDirection = Constants.PivotConstants.canCoderInvert;

        config.MagnetSensor = pivotCANcoderConfigs;
        return config;
    }

    public static CANcoderConfiguration clawPivotCancoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs clawCANcoderConfigs = new MagnetSensorConfigs();
        clawCANcoderConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        clawCANcoderConfigs.SensorDirection = Constants.ClawConstants.canCoderInvert;

        config.MagnetSensor = clawCANcoderConfigs;
        return config;
    }

    public static TalonFXConfiguration armPivotFXConfig() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        ClosedLoopRampsConfigs angleClosedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs angleOpenLoopRampsConfigs = new OpenLoopRampsConfigs();

        // angleSupplyLimit.StatorCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
        // angleSupplyLimit.StatorCurrentLimit = Constants.SwerveConstants.angleStatorCurrentLimit;
        // angleSupplyLimit.SupplyCurrentLimit = Constants.SwerveConstants.angleSupplyCurrentLimit;
        // angleSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleSupplyCurrentLimitEnable;

        // angleClosedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        // angleOpenLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        angleConfig.Slot0.kP = Constants.PivotConstants.kP;
        angleConfig.Slot0.kI = Constants.PivotConstants.kI;
        angleConfig.Slot0.kD = Constants.PivotConstants.kD;

        angleConfig.MotionMagic.MotionMagicAcceleration = 20;
        angleConfig.MotionMagic.MotionMagicCruiseVelocity = 10.6;

        angleConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.PivotConstants.kMaxForwardTorque;
        angleConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.PivotConstants.kMaxReverseTorque;


        // mPivot.config_kF(0, 0.045); // Is this supposed to be 0?


        // angleConfig.Slot0.kS = Constants.SwerveConstants.angleKS;
        // angleConfig.Slot0.kV = Constants.SwerveConstants.angleKV;
        angleConfig.CurrentLimits = angleSupplyLimit;
        angleConfig.OpenLoopRamps = angleOpenLoopRampsConfigs;
        angleConfig.ClosedLoopRamps = angleClosedLoopRampsConfigs;
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return angleConfig;
    }

    public static TalonFXConfiguration armFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        ClosedLoopRampsConfigs angleClosedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs angleOpenLoopRampsConfigs = new OpenLoopRampsConfigs();

        // angleSupplyLimit.StatorCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
        // angleSupplyLimit.StatorCurrentLimit = Constants.SwerveConstants.angleStatorCurrentLimit;
        // angleSupplyLimit.SupplyCurrentLimit = Constants.SwerveConstants.angleSupplyCurrentLimit;
        // angleSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleSupplyCurrentLimitEnable;

        // angleClosedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        // angleOpenLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        config.Slot0.kP = 0.6;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        // mArm.config_kF(0, 0.077);

        config.MotionMagic.MotionMagicAcceleration = 20;
        config.MotionMagic.MotionMagicCruiseVelocity = 10.6;


        config.CurrentLimits = angleSupplyLimit;
        config.OpenLoopRamps = angleOpenLoopRampsConfigs;
        config.ClosedLoopRamps = angleClosedLoopRampsConfigs;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        

        return config;
    }

}
