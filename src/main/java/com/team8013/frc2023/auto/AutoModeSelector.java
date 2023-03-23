package com.team8013.frc2023.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team8013.frc2023.auto.modes.*;
import com.team8013.frc2023.auto.modes.ConeToPickupToBalanceModes.*;
import com.team8013.frc2023.auto.modes.HighConeToBalanceModes.*;
import com.team8013.frc2023.auto.modes.StraightBackModes.*;
import com.team8013.frc2023.auto.modes.TwoPieceModes.*;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING,
        // CURVY_PATH,
        RIGHT_STRAIGHT_TO_BALANCE,
        LEFT_STRAIGHT_TO_BALANCE,
        CONE_MIDFIELD_STRAIGHT_TO_BALANCE,
        RIGHT_TO_PICKUP_TO_BALANCE,
        LEFT_TO_PICKUP_TO_BALANCE,
        RIGHT_TWO_PIECE,
        LEFT_TWO_PIECE,
        RIGHT_TWO_PIECE_TO_BALANCE,
        LEFT_TWO_PIECE_TO_BALANCE,
        RIGHT_CONE_EXIT,
        LEFT_CONE_EXIT,

    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        //mModeChooser.addOption("Curvy Path Mode", DesiredMode.CURVY_PATH);

        /*Pickup to balance Modes */
        mModeChooser.addOption("RIGHT TO PICKUP TO BALANCE Mode", DesiredMode.RIGHT_TO_PICKUP_TO_BALANCE);
        mModeChooser.addOption("LEFT TO PICKUP TO BALANCE Mode", DesiredMode.LEFT_TO_PICKUP_TO_BALANCE);

        /*Two Piece Modes */
        mModeChooser.addOption("RIGHT Two Piece Mode", DesiredMode.RIGHT_TWO_PIECE);
        mModeChooser.addOption("LEFT Two Piece Mode", DesiredMode.LEFT_TWO_PIECE);
        mModeChooser.addOption("RIGHT Two Piece To Balance Mode", DesiredMode.RIGHT_TWO_PIECE_TO_BALANCE);
        mModeChooser.addOption("LEFT Two Piece To Balance Mode", DesiredMode.LEFT_TWO_PIECE_TO_BALANCE);

        /*Straight to balance Modes */
        mModeChooser.addOption("RIGHT Straight To Balance Mode", DesiredMode.RIGHT_STRAIGHT_TO_BALANCE);
        mModeChooser.addOption("LEFT Straight To Balance Mode", DesiredMode.LEFT_STRAIGHT_TO_BALANCE);
        mModeChooser.addOption("CONE Mid Straight To Balance Mode", DesiredMode.CONE_MIDFIELD_STRAIGHT_TO_BALANCE);

        /*Straight Out Modes */
        mModeChooser.addOption("RIGHT Cone to exit Mode", DesiredMode.RIGHT_CONE_EXIT);
        mModeChooser.addOption("LEFT Cone to exit Mode", DesiredMode.LEFT_CONE_EXIT);


        SmartDashboard.putData(mModeChooser);
        //System.out.println("PUT AUTO MODE SELECTOR IN SMART DASHBOARD");
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

            // case CURVY_PATH:
            //     return Optional.of(new CurvyPathMode());

            /*Pickup To Balance Modes */
            case RIGHT_TO_PICKUP_TO_BALANCE:
                return Optional.of(new RightConeToPickupToBalanceMode());

            case LEFT_TO_PICKUP_TO_BALANCE:
                return Optional.of(new LeftConeToPickupToBalanceMode());

            /*Two Piece Modes */
            case RIGHT_TWO_PIECE:
                return Optional.of(new RightTwoPieceMode());

            case LEFT_TWO_PIECE:
                return Optional.of(new LeftTwoPieceMode());
            
            case RIGHT_TWO_PIECE_TO_BALANCE:
                return Optional.of(new RightTwoPieceToBalanceMode());

            case LEFT_TWO_PIECE_TO_BALANCE:
                return Optional.of(new LeftTwoPieceToBalanceMode());

            /*Straight to Balance Modes */
            case RIGHT_STRAIGHT_TO_BALANCE:
                return Optional.of(new RightConeStraightToBalance());

            case LEFT_STRAIGHT_TO_BALANCE:
                return Optional.of(new LeftConeStraightToBalance());

            case CONE_MIDFIELD_STRAIGHT_TO_BALANCE:
                return Optional.of(new MidConeStraightToBalance());

            /*Straight Out Modes */
            case RIGHT_CONE_EXIT:
                return Optional.of(new RightConeToStraightBack());

            case LEFT_CONE_EXIT:
                return Optional.of(new LeftConeToStraightBack());

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
