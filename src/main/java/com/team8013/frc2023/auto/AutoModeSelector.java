package com.team8013.frc2023.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team8013.frc2023.auto.modes.*;
import com.team8013.frc2023.shuffleboard.ShuffleBoardInteractions;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH_AUTO,
        NEW_2023_MODE
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Path Mode", DesiredMode.TEST_PATH_AUTO);
        mModeChooser.addOption("2023 Path Mode", DesiredMode.NEW_2023_MODE);
        // mModeChooser.addOption("Five Ball Mode", DesiredMode.FIVE_BALL_AUTO);
        
        SmartDashboard.putData(mModeChooser);
        //ShuffleBoardInteractions.getInstance().getOperatorTab().add("Auto Mode", mModeChooser).withSize(2, 1);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());

        case TEST_PATH_AUTO:
            return Optional.of(new TestPathMode());

        case NEW_2023_MODE:
            return Optional.of(new New2023Mode());

        // case ONE_BALL_RIGHT_AUTO:
        //     return Optional.of(new OneBallRightMode());

        // case TWO_BALL_AUTO:
        //     return Optional.of(new TwoBallMode());

        // case TWO_BY_ONE_AUTO:
        //     return Optional.of(new TwobyOneMode());

        // case TWO_BY_TWO_AUTO:
        //     return Optional.of(new TwobyTwoMode());

        // case FIVE_BALL_AUTO:
        //     return Optional.of(new FiveBallMode());
            
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}
