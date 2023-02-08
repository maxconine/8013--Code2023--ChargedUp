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
import com.team8013.frc2023.subsystems.ServoMotorSubsystem.ControlState;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Arm extends Subsystem {
    
    
    TalonFX motor;

    public Arm(TalonFX armMotor) {

        motor = armMotor;

    }

    public void resetPosition() {

        motor.setSelectedSensorPosition(0);

    }

    public double getPosition() {

        return motor.getSelectedSensorPosition();
    }

    public void maxLength() {

        while (-1 * (motor.getSelectedSensorPosition()) < 325000) {
            motor.set(TalonFXControlMode.PercentOutput, -.6);
        }

        motor.set(TalonFXControlMode.PercentOutput, 0);

    }

    public void minLength() {
        while (-1 * (motor.getSelectedSensorPosition()) > 0) {
            motor.set(TalonFXControlMode.PercentOutput, .6);
        }

        motor.set(TalonFXControlMode.PercentOutput, 0);

    }

}

