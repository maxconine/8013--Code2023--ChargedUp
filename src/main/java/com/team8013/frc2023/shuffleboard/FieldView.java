package com.team8013.frc2023.shuffleboard;

// import java.util.Optional;

// import com.team254.lib.vision.AimingParameters;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.RobotState;
// import com.team8013.frc2023.subsystems.Superstructure;
import com.team8013.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldView {
    private Field2d mField2d = new Field2d();
    private Swerve mSwerve = Swerve.getInstance();
    //private Superstructure mSuperstructure = Superstructure.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private Pose2d[] mModulePoses = new Pose2d[4];
    private Pose2d mRobotPose = new Pose2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void updateSwervePoses() {
        if(mSwerve.getPose() != null) mRobotPose = mSwerve.getPose();
        else mRobotPose = new Pose2d();
        for (int i = 0; i < mModulePoses.length; i++) {
            Translation2d updatedPosition = Constants.SwerveConstants.swerveModuleLocations[i]
                    .rotateBy(mRobotPose.getRotation()).plus(mRobotPose.getTranslation());
            Rotation2d updatedRotation = mSwerve.getStates()[i].angle.plus(mRobotPose.getRotation());
            if(mSwerve.getStates()[i].speedMetersPerSecond < 0.0) {
                updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));;
            }
            mModulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }
    }

    public void update() {
        updateSwervePoses();

        mField2d.setRobotPose(mRobotPose);
        mField2d.getObject("Swerve Modules").setPoses(mModulePoses);

        //if april tags present use that to update position

        mField2d.getObject("Predicted Robot Pose").setPose(mRobotState.getPredictedFieldToVehicle(Constants.VisionConstants.kLookaheadTime).getWpilibPose2d());
    }
}
