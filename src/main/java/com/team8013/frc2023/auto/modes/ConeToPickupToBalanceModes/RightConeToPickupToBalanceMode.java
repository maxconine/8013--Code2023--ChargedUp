package com.team8013.frc2023.auto.modes.ConeToPickupToBalanceModes;

import java.util.List;

import com.team8013.frc2023.Constants;
import com.team8013.frc2023.auto.AutoModeEndedException;
import com.team8013.frc2023.auto.AutoTrajectoryReader;
import com.team8013.frc2023.auto.actions.LambdaAction;
import com.team8013.frc2023.auto.actions.SeriesAction;
import com.team8013.frc2023.auto.actions.SwerveTrajectoryAction;
import com.team8013.frc2023.auto.actions.WaitAction;
import com.team8013.frc2023.auto.modes.AutoModeBase;
import com.team8013.frc2023.subsystems.Superstructure;
import com.team8013.frc2023.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class RightConeToPickupToBalanceMode extends AutoModeBase {

        // Swerve instance
        private final Swerve mSwerve = Swerve.getInstance();
        private final Superstructure mSuperstructure = Superstructure.getInstance();

        // required PathWeaver trajectory paths
        String path_a = "paths/RightConeToPickupToBalanceBlue_A.path";
        String path_b = "paths/RightConeToPickupToBalanceBlue_B.path";

        // trajectories
        SwerveTrajectoryAction farRightTopNodeToPickup_a;
        SwerveTrajectoryAction rightPickupToBalance_b;

        public RightConeToPickupToBalanceMode() {

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);

                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // read trajectories from PathWeaver and generate trajectory actions
                Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(path_a,
                                Constants.AutoConstants.defaultSpeedConfig);
                farRightTopNodeToPickup_a = new SwerveTrajectoryAction(traj_path_a,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(180),
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
                                () -> Rotation2d.fromDegrees(180),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

        }

        @Override
        protected void routine() throws AutoModeEndedException {
                System.out.println("Running top Cone to Balance auto!");

                // reset odometry at the start of the trajectory
                runAction(new LambdaAction(() -> mSwerve.resetOdometry(farRightTopNodeToPickup_a.getInitialPose())));

                // runAction(new WaitAction(0.5));

                runAction(new LambdaAction(() -> mSuperstructure.settingHighToDownAuto()));

                runAction(new WaitAction(Constants.AutoConstants.firstDropHighWait));

                //lets see if these actions run in series
                runAction(new SeriesAction(List.of(farRightTopNodeToPickup_a,
                        new LambdaAction(() -> mSuperstructure.settingPickupAuto()))));
                
                // runAction(farRightTopNodeToPickup_a);
                // runAction(new LambdaAction(() -> mSuperstructure.settingPickupAuto()));

                runAction(new LambdaAction(() -> mSuperstructure.clampClawAuto()));

                runAction(new WaitAction(Constants.AutoConstants.pickupPieceWait));

                runAction(rightPickupToBalance_b);

                runAction(new WaitAction(0.5));

                System.out.println("charge station engaging");

                runAction(new LambdaAction(() -> mSuperstructure.engageChargeStation(true)));

                System.out.println("Finished auto!");
        }

        @Override
        public Pose2d getStartingPose() {
                return farRightTopNodeToPickup_a.getInitialPose();
        }
}