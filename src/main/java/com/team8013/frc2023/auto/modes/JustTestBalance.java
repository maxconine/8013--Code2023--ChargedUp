package com.team8013.frc2023.auto.modes;

import com.team8013.frc2023.auto.AutoModeEndedException;

import com.team8013.frc2023.auto.actions.LambdaAction;

import com.team8013.frc2023.subsystems.Superstructure;
import com.team8013.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class JustTestBalance extends AutoModeBase {

    // Swerve instance
    private final Swerve mSwerve = Swerve.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    // required PathWeaver trajectory paths
    // String path_a = "paths/FarRightConeToBalanceBlue.path";

    // trajectories
    // SwerveTrajectoryAction rightConeToBalanceBlue;

    public JustTestBalance() {

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running top Cone to Balance auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));

        // runAction(new WaitAction(0.5));

        // runAction(new LambdaAction(() -> mSuperstructure.settingHighToDownAuto()));
        // runA

        System.out.println("charge station engaging");

        runAction(new LambdaAction(() -> mSuperstructure.engageChargeStation(true)));

        System.out.println("Finished auto!");
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(0, 0, new Rotation2d(0));
    }
}
