// package com.team8013.frc2023.subsystems;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenixpro.hardware.Pigeon2;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// //import edu.wpi.first.wpiutil.math.MathUtil;

// public class Claw {
// PowerDistribution revPower = new PowerDistribution(Constants.PDP_ID,
// ModuleType.kRev);

// PIDController m_PivotPid;
// VictorSPX m_PivotMotor;
// Encoder m_PivotEncoder;

// PIDController m_GripPid;
// VictorSPX m_GripMotor;
// Encoder m_GripEncoder;

// boolean closing = false;

// public Claw() {
// m_PivotEncoder = new Encoder(2, 3);
// m_PivotEncoder.setDistancePerPulse(1 / 44.4 / (86 / 56));
// m_PivotMotor = new VictorSPX(23);

// m_GripEncoder = new Encoder(0, 1);
// m_GripEncoder.setDistancePerPulse(1 / 44.4 / (86 / 56));
// m_GripMotor = new VictorSPX(24);

// m_PivotPid = new PIDController(2, 0, 0);
// m_GripPid = new PIDController(2, 0, 0);
// }

// public void drivePivot(double desiredRotation) {

// double currentDegrees = m_PivotEncoder.getDistance() * 360;

// // difference of desired rotation(0-360) to current position (0 to 1440)
// double difference = (currentDegrees % 360) - desiredRotation;

// double want = 1000;
// if (difference > 180) {
// if (currentDegrees + (360 - difference) <= Constants.Pivot.MaxRotation) {
// want = (currentDegrees + (360 - difference));
// } else {
// want = (currentDegrees - difference);
// }

// } else if (difference > 0) {
// if (currentDegrees - difference >= Constants.Pivot.MinRotation) {
// want = (currentDegrees - difference);
// } else {
// want = (currentDegrees + (360 - difference));
// }

// } else if (difference < -180) {
// if (currentDegrees - (360 + difference) >= Constants.Pivot.MinRotation) { //
// -abs(difference)
// want = (currentDegrees - (360 + difference));
// } else {
// want = (currentDegrees - difference); // +abs(difference)
// }
// } else if (difference < 0) {
// if (currentDegrees - difference <= Constants.Pivot.MaxRotation) {
// want = (currentDegrees - difference);
// } else {
// want = (currentDegrees - (360 + difference));
// }
// }
// if (want != 1000) {
// m_PivotPid.setSetpoint(want / 360); // should just put this where ever the
// value
// // of want is being changed
// }
// }

// // sets motor to specific number of rotations (dependent on current
// position;ex.
// // if pos=2, input=3, rotates once)

// // private void setPivotPosition(double rotation) {
// // SmartDashboard.putNumber("setPivPos", rotation /
// // Constants.Pivot.pivotGearRatio * 360);
// // System.out.println(rotation / Constants.Pivot.pivotGearRatio * 360);
// // m_PivotPidController.setReference(rotation,
// // CANSparkMax.ControlType.kPosition);
// // }

// // // output: current position in degrees on a unit cirlce
// // public double getPivotPositionDegrees() {
// // return m_PivotPivEncoder.getPosition() / Constants.Pivot.pivotGearRatio *
// // 360;
// // }

// public void resetPivotPosition() {
// m_PivotPid.setSetpoint(0);
// }

// public void resetPivotEncoder() {
// m_PivotEncoder.reset();
// }

// public void setGripPosition(double position) {
// m_GripPid.setSetpoint(position);
// }

// public void resetGripEncoder() {
// m_GripEncoder.reset();
// }

// public void openGrip() {
// m_GripPid.setSetpoint(.125);
// closing = false;// remove this in future and do it when it stops
// }

// public void closeGrip() {
// m_GripPid.setSetpoint(0);
// // m_GripMotor.set(VictorSPXControlMode.PercentOutput, 0.5);
// // closing = true;
// }

// public void keepCalling() {
// SmartDashboard.putNumber("pivot", m_PivotEncoder.getDistance());
// SmartDashboard.putNumber("grip", m_GripEncoder.getDistance());
// // m_PivotMotor.set(VictorSPXControlMode.PercentOutput,
// // MathUtil.clamp(m_PivotPid.calculate(m_PivotEncoder.getDistance()), -.25,
// // .25));
// // if (closing == true) {
// // double current = revPower.getCurrent(19);
// // System.out.println(current);
// // if (current >= 1) { //add timer thing

// // System.out.println(current);
// // m_GripMotor.set(VictorSPXControlMode.PercentOutput,0);
// // closing=false;
// // }
// // } else {
// // System.out.println(m_GripEncoder.getDistance());
// // m_GripMotor.set(VictorSPXControlMode.PercentOutput,
// // MathUtil.clamp(m_GripPid.calculate(m_GripEncoder.getDistance()), -1, 1));
// // }
// }
// }