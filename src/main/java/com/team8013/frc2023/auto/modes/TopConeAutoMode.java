package com.team8013.frc2023.auto.modes;



import com.team8013.frc2023.Constants;
import com.team8013.frc2023.auto.AutoModeEndedException;
import com.team8013.frc2023.auto.AutoTrajectoryReader;
import com.team8013.frc2023.auto.actions.LambdaAction;
import com.team8013.frc2023.auto.actions.SwerveTrajectoryAction;
import com.team8013.frc2023.auto.actions.WaitAction;
// import com.team8013.frc2023.subsystems.Superstructure;
import com.team8013.frc2023.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TopConeAutoMode extends AutoModeBase {

        // Swerve instance
        private final Swerve mSwerve = Swerve.getInstance();
        //private final Superstructure mSuperstructure = Superstructure.getInstance();

        // required PathWeaver file paths
        String file_path_a = "paths/TopConePaths/TopConeAuto_A.path";
        String file_path_b = "paths/TopConePaths/TopConeAuto_B.path";
        String file_path_c = "paths/TopConePaths/TopConeAuto_C.path";
        String file_path_d = "paths/TopConePaths/TopConeAuto_D.path";
        String file_path_e = "paths/TopConePaths/TopConeAuto_E.path";

        // trajectories
        private Trajectory traj_path_a;
        private Trajectory traj_path_b;
        private Trajectory traj_path_c;
        private Trajectory traj_path_d;
        private Trajectory traj_path_e;


        // trajectory actions
        SwerveTrajectoryAction driveToIntakeSecondCone;
        SwerveTrajectoryAction driveToNextTopConeSpot;
        SwerveTrajectoryAction driveToIntakeThirdCone;
        SwerveTrajectoryAction driveToLastTopConeSpot;
        SwerveTrajectoryAction driveOutOfCommunity;

        public TopConeAutoMode() {

                SmartDashboard.putBoolean("Auto Finished", false);

                // define theta controller for robot heading
                var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                /* CREATE TRAJECTORIES FROM FILES */

                // Intake second cargo
                traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeSecondCone = new SwerveTrajectoryAction(traj_path_a,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(206),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to lineup to third cargo
                traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToNextTopConeSpot = new SwerveTrajectoryAction(traj_path_b,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Intake third cargo
                traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeThirdCone = new SwerveTrajectoryAction(traj_path_c,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(199.4),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                traj_path_d = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_d,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToLastTopConeSpot = new SwerveTrajectoryAction(traj_path_d,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);
                                
                traj_path_e = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_e,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveOutOfCommunity = new SwerveTrajectoryAction(traj_path_e,
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
                System.out.println("Running top cone mode auto!");
                SmartDashboard.putBoolean("Auto Finished", false);

                // disable auto ejecting
                runAction(new WaitAction(.50));

                runAction(new LambdaAction(() -> mSwerve.resetOdometry(driveToIntakeSecondCone.getInitialPose())));

                runAction(driveToIntakeSecondCone);

                runAction(new WaitAction(.5));
                // start vision aiming
                //runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

                // run trajectory for third cargo
                runAction(driveToNextTopConeSpot);

                runAction(new WaitAction(0.5));
                // shoot first & second cargo
                //runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
                //runAction(new WaitAction(1.0));
                //runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

                // run trajectory to intake third shot cargo
                runAction(driveToIntakeThirdCone);

                runAction(new WaitAction(0.5));

                runAction(driveToLastTopConeSpot);

                runAction(new WaitAction(1));

                runAction(driveOutOfCommunity);


                // stop vision aiming to control robot heading
                //runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

                System.out.println("Finished auto!");
                SmartDashboard.putBoolean("Auto Finished", true);
        }

        @Override
        public Pose2d getStartingPose() {
                return driveToIntakeThirdCone.getInitialPose();
        }
}

