package com.team8013.frc2023.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.team8013.frc2023.auto.modes.*;
import com.team8013.frc2023.shuffleboard.ShuffleBoardInteractions;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH_AUTO,
        TOP_CONE_MODE,
        DRIVE_FORWARD,
        CURVY_PATH
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Path Mode", DesiredMode.TEST_PATH_AUTO);
        mModeChooser.addOption("Top Cone Mode", DesiredMode.TOP_CONE_MODE);
        mModeChooser.addOption("Drive Forward Mode", DesiredMode.DRIVE_FORWARD);
        mModeChooser.addOption("Curvy Path Mode", DesiredMode.CURVY_PATH);
        // mModeChooser.addOption("Five Ball Mode", DesiredMode.FIVE_BALL_AUTO);
        
        SmartDashboard.putData(mModeChooser);
        System.out.println("PUT AUTO MODE SELECTOR IN SMART DASHBOARD");
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

        case TOP_CONE_MODE:
            return Optional.of(new TopConeAutoMode());

        case DRIVE_FORWARD:
            return Optional.of(new DriveForward());

        case CURVY_PATH:
            return Optional.of(new CurvyPathMode());

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
