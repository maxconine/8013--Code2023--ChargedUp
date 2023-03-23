package com.team8013.frc2023.shuffleboard.tabs;

import com.team8013.frc2023.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2023.subsystems.Superstructure;

import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SuperstructureTab extends ShuffleboardTabBase {

	private Superstructure mSuperstructure = Superstructure.getInstance();

	private GenericEntry mEngage;
    private GenericEntry clampClawAuto;
    private GenericEntry hasClosedGrip;
	private GenericEntry wantDropPieceAuto;

	private GenericEntry wantedYellow;
    private GenericEntry wantedBlue;

    private GenericEntry openingClaw;
	private GenericEntry maxArmPosition;
	private GenericEntry mAdjustedRoll;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Superstructure");
		
		mEngage = mTab
            .add("Auto Engage", false)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
			
		clampClawAuto = mTab
            .add("clampClawAuto", false)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
			
		hasClosedGrip = mTab
            .add("hasClosedGrip Auto", false)
            .withPosition(3, 0)
            .withSize(2, 1)
            .getEntry();
			
		wantDropPieceAuto = mTab
            .add("wantDropPieceAuto", false)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();

		wantedYellow = mTab
			.add("wantedYellow", false)
			.withPosition(0, 1)
			.withSize(2, 1)
			.getEntry();

		wantedBlue = mTab
			.add("wantedBlue", false)
			.withPosition(1, 1)
			.withSize(2, 1)
			.getEntry();
			
		openingClaw = mTab
			.add("openingClaw", false)
			.withPosition(0, 2)
			.withSize(2, 1)
			.getEntry();

		maxArmPosition = mTab
			.add("maxArmPosition", 0)
			.withPosition(1, 2)
			.withSize(2, 1)
			.getEntry();
		mAdjustedRoll = mTab
			.add("Adjusted Roll", 0)
			.withPosition(2, 2)
			.withSize(2, 1)
			.getEntry();
	}

	@Override
	public void update() {

		mEngage.setBoolean(mSuperstructure.mPeriodicIO.mEngage);
		clampClawAuto.setBoolean(mSuperstructure.mPeriodicIO.clampClawAuto);
		hasClosedGrip.setBoolean(mSuperstructure.mPeriodicIO.hasClosedGrip);
		wantDropPieceAuto.setBoolean(mSuperstructure.mPeriodicIO.wantDropPieceAuto);
		wantedYellow.setBoolean(mSuperstructure.mPeriodicIO.wantedYellow);
		wantedBlue.setBoolean(mSuperstructure.mPeriodicIO.wantedBlue);
		openingClaw.setBoolean(mSuperstructure.mPeriodicIO.openingClaw);
		maxArmPosition.setDouble(truncate(mSuperstructure.mPeriodicIO.maxArmPosition));
		mAdjustedRoll.setDouble(truncate(mSuperstructure.getAdjustedRoll()));

	}

}
