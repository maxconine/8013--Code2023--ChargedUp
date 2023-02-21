package com.team8013.frc2023.drivers;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMaxRelativeEncoder;
//import frc.robot.Constants;

public class NeoMotor {
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    /**
     * A RelativeEncoder object is constructed using the GetEncoder() method on an
     * existing CANSparkMax object. The assumed encoder type is the hall effect,
     * or a sensor type and counts per revolution can be passed in to specify
     * a different kind of sensor. Here, it's a quadrature encoder with 4096 CPR.
     */
    public RelativeEncoder m_encoder;

    public NeoMotor(CANSparkMax canMotor, double kPvalue, double kIvalue, double kDvalue, double kMax, double kMin) {
        // initialize SPARK MAX with CAN ID
        m_motor = canMotor;
        m_encoder = m_motor.getEncoder();

        m_motor.restoreFactoryDefaults();

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController
         * object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = m_motor.getPIDController();

        /**
         * The PID Controller can be configured to use the analog sensor as its feedback
         * device with the method SetFeedbackDevice() and passing the PID Controller
         * the CANAnalog object.
         */
        m_pidController.setFeedbackDevice(m_encoder);

        // PID coefficients
        kP = kPvalue;
        kI = kIvalue;
        kD = kDvalue;
        kIz = 0;
        kFF = 0;
        kMaxOutput = kMax;
        kMinOutput = kMin;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);
        // SmartDashboard.putNumber("Set Rotations", 0);

    }

    public void setPID(double kPvalue, double kIvalue, double kDvalue) {
        // read PID coefficients from SmartDashboard
        double p = kPvalue;
        double i = kIvalue;
        double d = kDvalue;

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            m_pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pidController.setD(d);
            kD = d;
        }

        /**
         * PIDController objects are commanded to a set point using the
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four
         * parameters:
         * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         * com.revrobotics.CANSparkMax.ControlType.kPosition
         * com.revrobotics.CANSparkMax.ControlType.kVelocity
         * com.revrobotics.CANSparkMax.ControlType.kVoltage
         */

        // SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
    }

    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
    }

    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public void setPosition(double input) {
        m_pidController.setReference(input, CANSparkMax.ControlType.kPosition);
    }

    public double getMotorOutputVoltage() {
        return m_motor.getBusVoltage();
    }

    public double getMotorVelocityRPM() {
        return m_encoder.getVelocity();
    }

}
