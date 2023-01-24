package com.team8013.frc2023.shuffleboard.tabs;

import com.team8013.frc2023.shuffleboard.ShuffleboardTabBase;
// import com.team8013.frc2023.subsystems.Superstructure;

// import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SuperstructureTab extends ShuffleboardTabBase {

	// private Superstructure mSuperstructure = Superstructure.getInstance();

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Superstructure");
		// actions
	}

	@Override
	public void update() {
        // mIsAimed.setBoolean(mSuperstructure.isAimed());
	}

}
