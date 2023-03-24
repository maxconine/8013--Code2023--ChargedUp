package com.team8013.frc2023.shuffleboard.tabs;

import com.team8013.frc2023.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2023.subsystems.Arm;

import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ArmTab extends ShuffleboardTabBase {

	private Arm mArm = Arm.getInstance();

	private GenericEntry mArmVelocity;

	private GenericEntry mArmDemand;

	private GenericEntry mArmPosition;
	private GenericEntry arm_maxTravel;

	private GenericEntry mArmCurrent;

	private GenericEntry isPulledIn;

	private GenericEntry mArmControlState;

	private GenericEntry pullArmIntoZero;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Arm");
		mArmControlState = mTab
				.add("Arm  Control State", mArm.mArmControlState.toString())
				.getEntry();
		mArmVelocity = mTab
				.add("Arm Velocity", 0.0)
				.getEntry();
		mArmDemand = mTab
				.add("Arm Demand", 0.0)
				.getEntry();
		mArmPosition = mTab
				.add("Arm Position", 0.0)
				.getEntry();
		mArmCurrent = mTab
				.add("Arm Current", 0.0)
				.getEntry();
		isPulledIn = mTab
				.add("Arm Pulled In", false)
				.getEntry();
		pullArmIntoZero = mTab
				.add("Arm Zeroing", false)
				.getEntry();
		arm_maxTravel = mTab
				.add("Arm Max Travel", 0.0)
				.getEntry();

	}

	@Override
	public void update() {
		pullArmIntoZero.setBoolean(mArm.getArmPullInToZero());

		mArmControlState.setString(mArm.getControlState().toString());

		mArmVelocity.setDouble(mArm.getArmVelocity());

		mArmDemand.setDouble(mArm.getArmDemand());

		mArmPosition.setDouble(mArm.getArmPosition());

		mArmCurrent.setDouble(mArm.getArmCurrent());

		isPulledIn.setBoolean(mArm.isIn());

		arm_maxTravel.setDouble(mArm.getArmMaxTravel());
	}

}
