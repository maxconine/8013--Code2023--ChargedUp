package com.team8013.frc2023.shuffleboard.tabs;


import com.team8013.frc2023.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2023.subsystems.ClawV2;

import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ClawTab extends ShuffleboardTabBase {

	private ClawV2 mClaw = ClawV2.getInstance();

	private GenericEntry mClawGripVelocity;
    private GenericEntry mClawPivotVelocity;

	private GenericEntry mClawGripDemand;
    private GenericEntry mClawPivotDemand;

	private GenericEntry mClawGripPosition;
	private GenericEntry mClawPivotPosition;

	private GenericEntry mClawGripCurrent;
    private GenericEntry mClawPivotCurrent;

	private GenericEntry mGripPeakSpeed;

	private GenericEntry mGripControlState;
    private GenericEntry mPivotControlState;

	private GenericEntry mWantedClosing;
    private GenericEntry mLimitSwitchActivated;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Claw");
		mGripControlState = mTab
				.add("Claw Grip  Control State", mClaw.mGripControlState.toString())
				.getEntry();
        mPivotControlState = mTab
				.add("Claw Pivot  Control State", mClaw.mPivotControlState.toString())
				.getEntry();
        
        mClawGripVelocity = mTab
				.add("Claw Grip Velocity", 0.0)
				.getEntry();
        mClawPivotVelocity = mTab
				.add("Claw Pivot Velocity", 0.0)
				.getEntry();
		
        mClawGripDemand = mTab
				.add("Claw Grip Demand", 0.0)
				.getEntry();
        mClawPivotDemand = mTab
				.add("Claw Pivot Demand", 0.0)
				.getEntry();
        
        mClawGripPosition = mTab
				.add("Claw Grip Position", 0.0)
				.getEntry();
        mClawPivotPosition = mTab
				.add("Claw Pivot CANCoder", 0.0)
				.getEntry();
        
		mClawGripCurrent = mTab
				.add("Claw Grip Current", 0.0)
				.getEntry();
        mClawPivotCurrent = mTab
				.add("Claw Pivot Current", 0.0)
				.getEntry();

        mGripPeakSpeed = mTab
				.add("Claw Grip Peak Speed", 0.0)
				.getEntry();
        
        mWantedClosing = mTab
				.add("Claw Wanted Closing", false)
				.getEntry();
        mLimitSwitchActivated = mTab
				.add("Claw Limit Switch Activated", false)
				.getEntry();
	}

	@Override
	public void update() {
		mGripControlState.setString(mClaw.getClawGripControlState().toString());
        mPivotControlState.setString(mClaw.getClawPivotControlState().toString());

        mClawGripVelocity.setDouble(mClaw.getGripVelocity());
        mClawPivotVelocity.setDouble(mClaw.getPivotVelocity());
        
        mClawGripDemand.setDouble(mClaw.getGripDemand());
        mClawPivotDemand.setDouble(mClaw.getPivotDemand());

        mClawGripPosition.setDouble(mClaw.getGripPosition());
        mClawPivotPosition.setDouble(mClaw.getPivotPosition());

        mClawGripCurrent.setDouble(mClaw.getGripCurrent());
        mClawPivotCurrent.setDouble(mClaw.getPivotCurrent());
        
        mLimitSwitchActivated.setBoolean(mClaw.getLimitSwitch());

        mWantedClosing.setBoolean(mClaw.wantedClosing());

		mGripPeakSpeed.setDouble(mClaw.getGripPeakSpeed());
		
	}

}
