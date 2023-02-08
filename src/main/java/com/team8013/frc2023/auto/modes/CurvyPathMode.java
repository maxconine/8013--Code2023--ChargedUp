package com.team8013.frc2023.auto.modes;


import com.team8013.frc2023.Constants;
import com.team8013.frc2023.auto.AutoModeEndedException;
import com.team8013.frc2023.auto.AutoTrajectoryReader;
import com.team8013.frc2023.auto.actions.LambdaAction;
import com.team8013.frc2023.auto.actions.SwerveTrajectoryAction;
import com.team8013.frc2023.auto.actions.WaitAction;
import com.team8013.frc2023.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class CurvyPathMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve mSwerve = Swerve.getInstance();

    // required PathWeaver trajectory paths
    String path_a = "paths/Turn2x2m.path";
    String path_b = "paths/TurnBack2x2m.path";
    
	// trajectories
	SwerveTrajectoryAction testTrajectoryAction_a;
    SwerveTrajectoryAction testTrajectoryAction_b;

    public CurvyPathMode() {

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    
        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(path_a, Constants.AutoConstants.defaultSpeedConfig);
        testTrajectoryAction_a = new SwerveTrajectoryAction(traj_path_a,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(-90),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);
		
    

            // read trajectories from PathWeaver and generate trajectory actions
            Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(path_b, Constants.AutoConstants.defaultSpeedConfig);
            testTrajectoryAction_b = new SwerveTrajectoryAction(traj_path_b,
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
        System.out.println("Running curvy path auto!");

        runAction(new WaitAction(0.5));

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(testTrajectoryAction_a.getInitialPose())));

        System.out.println("reset odometry action run");
        
        runAction(testTrajectoryAction_a);

        runAction(new WaitAction(2));

        runAction(testTrajectoryAction_b);
        
        System.out.println("Finished auto!");
    }

    @Override
    public Pose2d getStartingPose() {
        return testTrajectoryAction_a.getInitialPose();
    }
}


