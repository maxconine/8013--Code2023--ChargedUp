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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class New2023Mode extends AutoModeBase {

   // Swerve instance 
   private final Swerve mSwerve = Swerve.getInstance();
   private final Superstructure mSuperstructure = Superstructure.getInstance(); 

   // required PathWeaver file paths
   String file_path_a = "paths/New2023Paths/path_A.path";
   String file_path_b = "paths/New2023Paths/path_B.path";
   String file_path_c = "paths/New2023Paths/path_C.path";

   //trajectory actions
   SwerveTrajectoryAction driveForward;
   SwerveTrajectoryAction turnAround;
   SwerveTrajectoryAction driveBack;
   
   public New2023Mode() {

       SmartDashboard.putBoolean("Auto Finished", false);

       // define theta controller for robot heading
       var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
       thetaController.enableContinuousInput(-Math.PI, Math.PI);
       
       // read trajectories from PathWeaver and generate trajectory actions
       Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a, Constants.AutoConstants.defaultSpeedConfig);
       driveForward = new SwerveTrajectoryAction(traj_path_a,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(0.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);

        // read trajectories from PathWeaver and generate trajectory actions
       Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.defaultSpeedConfig);
       turnAround = new SwerveTrajectoryAction(traj_path_b,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(0.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);

       Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c, Constants.AutoConstants.defaultSpeedConfig);
       driveBack = new SwerveTrajectoryAction(traj_path_c,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(0.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);
   }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running New 2023 Mode!");
        SmartDashboard.putBoolean("Auto Finished", false);

    // wait for modules to align
    runAction(new WaitAction(0.5));

    // drive forward a few feet
    runAction(driveForward);

    
    // wait 2 sec
    runAction(new WaitAction(2));

    // turn around
    runAction(turnAround);

    //wait another 2 sec
    runAction(new WaitAction(2.0));

    // drive backwards to starting point
    runAction(driveBack);

    // ready for teleop
    runAction(new LambdaAction(() -> mSuperstructure.setInitialTeleopStates()));

    System.out.println("Finished auto!");
    SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return driveForward.getInitialPose();
    }

}
