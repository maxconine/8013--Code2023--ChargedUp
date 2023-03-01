package com.team8013.frc2023.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.team8013.frc2023.auto.modes.*;
import com.team8013.frc2023.auto.modes.HighConeToBalanceModes.*;
import com.team8013.frc2023.auto.modes.HighConeToBalanceModes.HighConeMidfieldToBalance;
import com.team8013.frc2023.auto.modes.HighConeToBalanceModes.HighConeToBalanceMode;
import com.team8013.frc2023.auto.modes.HighConeToBalanceModes.RightConeStraightToBalance;
import com.team8013.frc2023.auto.modes.StraightBackModes.LeftConeToStraightBack;
import com.team8013.frc2023.auto.modes.StraightBackModes.RightConeToStraightBack;
import com.team8013.frc2023.shuffleboard.ShuffleBoardInteractions;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING,
        CURVY_PATH,
        HIGH_CONE_TO_BALANCE,
        CONE_RIGHT_STRAIGHT_TO_BALANCE,
        CONE_LEFT_STRAIGHT_TO_BALANCE,
        RIGHT_CONE_EXIT_LAST_SECOND,
        LEFT_CONE_EXIT_LAST_SECOND,
        HIGH_CONE_MIDFIELD_TO_BALANCE
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Curvy Path Mode", DesiredMode.CURVY_PATH);
        mModeChooser.addOption("High Cone To Balance Mode", DesiredMode.HIGH_CONE_TO_BALANCE);
        mModeChooser.addOption("Cone RIGHT Straight To Balance Mode", DesiredMode.CONE_RIGHT_STRAIGHT_TO_BALANCE);
        mModeChooser.addOption("Cone LEFT Straight To Balance Mode", DesiredMode.CONE_LEFT_STRAIGHT_TO_BALANCE);
        mModeChooser.addOption("High Cone RIGHT to last second exit Mode",
                DesiredMode.RIGHT_CONE_EXIT_LAST_SECOND);
        mModeChooser.addOption("High Cone LEFT to last second exit Mode",
                DesiredMode.LEFT_CONE_EXIT_LAST_SECOND);
        mModeChooser.addOption("High Cone Mid TO Balance Mode",
                DesiredMode.HIGH_CONE_MIDFIELD_TO_BALANCE);

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

            case CURVY_PATH:
                return Optional.of(new CurvyPathMode());

            case HIGH_CONE_TO_BALANCE:
                return Optional.of(new HighConeToBalanceMode());

            case CONE_RIGHT_STRAIGHT_TO_BALANCE:
                return Optional.of(new RightConeStraightToBalance());

            case CONE_LEFT_STRAIGHT_TO_BALANCE:
                return Optional.of(new LeftConeStraightToBalance());

            case LEFT_CONE_EXIT_LAST_SECOND:
                return Optional.of(new LeftConeToStraightBack());

            case RIGHT_CONE_EXIT_LAST_SECOND:
                return Optional.of(new RightConeToStraightBack());

            case HIGH_CONE_MIDFIELD_TO_BALANCE:
                return Optional.of(new HighConeMidfieldToBalance());

            // case FIVE_BALL_AUTO:
            // return Optional.of(new FiveBallMode());

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
