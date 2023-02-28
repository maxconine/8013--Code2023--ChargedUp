package com.team8013.frc2023.auto.modes;

import com.team8013.frc2023.Constants;
import com.team8013.frc2023.auto.AutoModeEndedException;
import com.team8013.frc2023.auto.AutoTrajectoryReader;
import com.team8013.frc2023.auto.actions.LambdaAction;
import com.team8013.frc2023.auto.actions.SwerveTrajectoryAction;
import com.team8013.frc2023.auto.actions.WaitAction;
import com.team8013.frc2023.subsystems.Superstructure;
import com.team8013.frc2023.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class HighConeToBalanceMode extends AutoModeBase {

    // Swerve instance
    private final Swerve mSwerve = Swerve.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    // required PathWeaver trajectory paths
    String path_a = "paths/FarRightTopNodeToPickup.path";
    String path_b = "paths/RightPickupToBalance.path";

    // trajectories
    SwerveTrajectoryAction farRightTopNodeToPickup_a;
    SwerveTrajectoryAction rightPickupToBalance_b;

    public HighConeToBalanceMode() {

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(path_a,
                Constants.AutoConstants.defaultSpeedConfig);
        farRightTopNodeToPickup_a = new SwerveTrajectoryAction(traj_path_a,
                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                mSwerve::getWantAutoVisionAim,
                mSwerve::setModuleStates);

        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(path_b,
                Constants.AutoConstants.defaultSpeedConfig);
        rightPickupToBalance_b = new SwerveTrajectoryAction(traj_path_b,
                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                mSwerve::getWantAutoVisionAim,
                mSwerve::setModuleStates);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running top Cone to Balance auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(farRightTopNodeToPickup_a.getInitialPose())));

        // runAction(new WaitAction(0.5));

        runAction(new LambdaAction(() -> mSuperstructure.pivUpAuto()));

        runAction(new WaitAction(2));

        runAction(new LambdaAction(() -> mSuperstructure.armExtendHighAuto()));

        runAction(new WaitAction(2));

        runAction(new LambdaAction(() -> mSuperstructure.dropConeAuto()));

        runAction(new WaitAction(1));

        System.out.println("reset odometry action run");

        runAction(new LambdaAction(() -> mSuperstructure.pullArmInAuto()));

        runAction(new WaitAction(2));

        runAction(new LambdaAction(() -> mSuperstructure.pivPickupAuto()));

        runAction(farRightTopNodeToPickup_a);

        runAction(new LambdaAction(() -> mSuperstructure.clampCube()));

        runAction(new LambdaAction(() -> mSuperstructure.pullArmInAuto()));

        runAction(new WaitAction(1));

        runAction(new LambdaAction(() -> mSuperstructure.pullPivInAuto()));

        runAction(rightPickupToBalance_b);

        runAction(new LambdaAction(() -> mSuperstructure.engageChargeStation()));

        System.out.println("Finished auto!");
    }

    @Override
    public Pose2d getStartingPose() {
        return farRightTopNodeToPickup_a.getInitialPose();
    }
}
