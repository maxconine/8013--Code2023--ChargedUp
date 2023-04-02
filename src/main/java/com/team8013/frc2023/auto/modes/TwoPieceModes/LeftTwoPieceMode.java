package com.team8013.frc2023.auto.modes.TwoPieceModes;

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

public class LeftTwoPieceMode extends AutoModeBase {

        // Swerve instance
        private final Swerve mSwerve = Swerve.getInstance();
        private final Superstructure mSuperstructure = Superstructure.getInstance();

        // required PathWeaver trajectory paths
        String path_a = "paths/LeftTwoPieceBlue_A.path";
        String path_b = "paths/LeftTwoPieceBlue_B.path";
        String path_c = "paths/LeftTwoPieceBlue_C.path";
        String path_d = "paths/LeftTwoPieceBlue_D.path";

        // trajectories
        SwerveTrajectoryAction leftTwoPieceBlue_a;
        SwerveTrajectoryAction leftTwoPieceBlue_b;
        SwerveTrajectoryAction leftTwoPieceBlue_c;
        SwerveTrajectoryAction leftTwoPieceBlue_d;

        public LeftTwoPieceMode() {

                var thetaController1 = new ProfiledPIDController(
                                0, 0, 0.0,
                                Constants.AutoConstants.kThetaControllerConstraints);

                var thetaController2 = new ProfiledPIDController(
                                1.55, 0, 0.2,
                                Constants.AutoConstants.kThetaControllerConstraints);

                thetaController1.enableContinuousInput(-Math.PI, Math.PI);
                thetaController2.enableContinuousInput(-Math.PI, Math.PI);

                // read trajectories from PathWeaver and generate trajectory actions
                Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(path_a,
                                Constants.AutoConstants.defaultSpeedConfig);
                leftTwoPieceBlue_a = new SwerveTrajectoryAction(traj_path_a,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController1,
                                () -> Rotation2d.fromDegrees(180), // -72 degrees
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(path_b,
                                Constants.AutoConstants.leftTwoPieceConfig);
                leftTwoPieceBlue_b = new SwerveTrajectoryAction(traj_path_b,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController2,
                                () -> Rotation2d.fromDegrees(1), // -72 degrees
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // read trajectories from PathWeaver and generate trajectory actions
                Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(path_c,
                                Constants.AutoConstants.defaultSpeedConfig);
                leftTwoPieceBlue_c = new SwerveTrajectoryAction(traj_path_c,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController1,
                                () -> Rotation2d.fromDegrees(180),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                Trajectory traj_path_d = AutoTrajectoryReader.generateTrajectoryFromFile(path_d,
                                Constants.AutoConstants.slowSpeedConfig);
                leftTwoPieceBlue_d = new SwerveTrajectoryAction(traj_path_d,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController2,
                                () -> Rotation2d.fromDegrees(180), // -72 degrees
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

        }

        @Override
        protected void routine() throws AutoModeEndedException {
                System.out.println("Running top Cone to Balance auto!");

                // reset odometry at the start of the trajectory
                runAction(new LambdaAction(() -> mSwerve.resetOdometry(leftTwoPieceBlue_a.getInitialPose())));

                runAction(new WaitAction(0.25));

                runAction(new LambdaAction(() -> mSuperstructure.settingHighToDownAuto()));
                runAction(new LambdaAction(() -> mSuperstructure.wantDropPieceAuto()));

                runAction(new WaitAction(Constants.AutoConstants.firstDropHighWait + 1));

                // lets see if these actions run in series
                // runAction(new SeriesAction(List.of(leftTwoPieceBlue_a,
                // new LambdaAction(() -> mSuperstructure.settingPickupAuto()))));

                // System.out.println("hello");
                runAction(leftTwoPieceBlue_a);
                runAction(leftTwoPieceBlue_b);

                runAction(new WaitAction(0.25));

                // runAction(leftTwoPieceBlue_b);

                runAction(new LambdaAction(() -> mSuperstructure.settingPickupAuto()));

                runAction(new LambdaAction(() -> mSuperstructure.clampClawAuto()));
                System.out.println("helloclamp");

                runAction(new WaitAction(Constants.AutoConstants.pickupPieceWait));

                runAction(new LambdaAction(() -> mSuperstructure.setDownAuto()));

                runAction(leftTwoPieceBlue_c);

                runAction(leftTwoPieceBlue_d);

                runAction(new LambdaAction(() -> mSuperstructure.settingCubeDropHigh()));

                // runAction(new LambdaAction(() -> mSuperstructure.wantDropPieceAuto()));

                // can change this to drop hybrid if it is short on time
                // runAction(new LambdaAction(() -> mSuperstructure.settingHighToDownAuto()));

                System.out.println("Finished auto!");

        }

        @Override
        public Pose2d getStartingPose() {
                return leftTwoPieceBlue_a.getInitialPose();
        }
}
