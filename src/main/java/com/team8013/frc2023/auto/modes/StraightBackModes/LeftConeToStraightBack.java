package com.team8013.frc2023.auto.modes.StraightBackModes;

import com.team8013.frc2023.Constants;
import com.team8013.frc2023.auto.AutoModeEndedException;
import com.team8013.frc2023.auto.AutoTrajectoryReader;
import com.team8013.frc2023.auto.actions.LambdaAction;

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

public class LeftConeToStraightBack extends AutoModeBase {

        // Swerve instance
        private final Swerve mSwerve = Swerve.getInstance();
        private final Superstructure mSuperstructure = Superstructure.getInstance();

        // required PathWeaver trajectory paths
        String path_a = "paths/LeftStraightOutBlue.path";

        // trajectories
        SwerveTrajectoryAction leftStraightOutBlue;

        public LeftConeToStraightBack() {

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);

                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // read trajectories from PathWeaver and generate trajectory actions
                Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(path_a,
                                Constants.AutoConstants.slowSpeedConfig);
                leftStraightOutBlue = new SwerveTrajectoryAction(traj_path_a,
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
                runAction(new LambdaAction(() -> mSwerve.resetOdometry(leftStraightOutBlue.getInitialPose())));

                runAction(new WaitAction(0.25));

                runAction(new LambdaAction(() -> mSuperstructure.settingHighToDownAuto()));
                runAction(new LambdaAction(() -> mSuperstructure.wantDropPieceAuto()));

                runAction(new WaitAction(Constants.AutoConstants.firstDropHighWait));

                // runAction(leftStraightOutBlue);

                System.out.println("Finished auto!");
        }

        @Override
        public Pose2d getStartingPose() {
                return leftStraightOutBlue.getInitialPose();
        }
}
