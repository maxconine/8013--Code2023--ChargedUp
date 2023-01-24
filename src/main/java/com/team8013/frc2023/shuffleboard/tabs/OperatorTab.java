package com.team8013.frc2023.shuffleboard.tabs;

import com.team8013.frc2023.shuffleboard.ShuffleboardTabBase;
import com.team8013.frc2023.subsystems.Limelight;
// import com.team8013.frc2023.subsystems.Superstructure;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class OperatorTab extends ShuffleboardTabBase {

    // private Superstructure mSuperstructure = Superstructure.getInstance();
    private Limelight mLimelight = Limelight.getInstance();

    // private GenericEntry mOperatorShooting;
    // private GenericEntry mOperatorSpunup;
    // private GenericEntry mOperatorFender;
    // private GenericEntry mOperatorSpit;
    private GenericEntry mOperatorVisionAimed;
    // private GenericEntry mOperatorClimbMode;
    // private GenericEntry mOperatorAutoClimb;
    // private GenericEntry mOperatorEjectDisable;
    // private GenericEntry mOperatorIntakeOverride;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Operator");

        mOperatorVisionAimed = mTab
                .add("Vision Aimed", false)
                .withSize(6, 2)
                .withPosition(2, 3)
                .getEntry();
    }

    @Override
    public void update() {
        // mOperatorShooting.setBoolean(mSuperstructure.getShooting());
        // mOperatorSpunup.setBoolean(mSuperstructure.isSpunUp());
        // mOperatorSpit.setBoolean(mSuperstructure.getWantsSpit());
        // mOperatorFender.setBoolean(mSuperstructure.getWantsFender());
         mOperatorVisionAimed.setBoolean(mLimelight.isAimed());
        // mOperatorClimbMode.setBoolean(mSuperstructure.getInClimbMode());
        // mOperatorAutoClimb.setBoolean(mSuperstructure.isAutoClimb());

        // mOperatorEjectDisable.setBoolean(mSuperstructure.getEjectDisabled());
        // mOperatorIntakeOverride.setBoolean(mSuperstructure.getIntakeOverride());
    }

}
