package com.team8013.frc2023.shuffleboard.tabs;

import com.team8013.frc2023.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2023.subsystems.LEDs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class LedTab extends ShuffleboardTabBase {

	private LEDs mLEDs = LEDs.getInstance();

	private GenericEntry mLEDState;


	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Leds");
		/* CANdle */
		mLEDState = mTab
				.add("LEDs State", "N/A")
				.withSize(2, 1)
				.getEntry();

	}

	@Override
	public void update() {
        mLEDState.setString(mLEDs.getState().getName());

	}

}
