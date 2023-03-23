package com.team8013.frc2023.shuffleboard.tabs;


import com.team8013.frc2023.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2023.subsystems.PivotV2;

import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class PivotTab extends ShuffleboardTabBase {

	private PivotV2 mPivot = PivotV2.getInstance();

	private GenericEntry mPivotVelocity;

	private GenericEntry mPivotDemand;

	private GenericEntry mPivotPosition;
	private GenericEntry pivot_cancoder;

	private GenericEntry mPivotCurrent;



	private GenericEntry mPivotControlState;



	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Pivot");
		mPivotControlState = mTab
				.add("Pivot  Control State", mPivot.mPivotControlState.toString())
				.getEntry();
		mPivotVelocity = mTab
				.add("Pivot Velocity", 0.0)
				.getEntry();
		mPivotDemand = mTab
				.add("Pivot Demand", 0.0)
				.getEntry();
		mPivotPosition = mTab
				.add("Pivot Position", 0.0)
				.getEntry();
		mPivotCurrent = mTab
				.add("Pivot Current", 0.0)
				.getEntry();
        pivot_cancoder =  mTab
			.add("Relative Pivot CANCODER", 0.0)
			.getEntry();

	}

	@Override
	public void update(){ 
		mPivotControlState.setString(mPivot.getControlState().toString());

        mPivotVelocity.setDouble(mPivot.getPivotVelocity());
        
        mPivotDemand.setDouble(mPivot.getPivotDemand());

        mPivotPosition.setDouble(mPivot.getPivotPosition());

        mPivotCurrent.setDouble(mPivot.getPivotCurrent());

        pivot_cancoder.setDouble(mPivot.getRelativeCancoder());
		
	}

}
