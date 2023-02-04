package com.team8013.frc2023.controlboard;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.controlboard.CustomXboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.stickDeadband;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private static ControlBoard mInstance = null;



    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private final GenericHID m_driver;
    public final CustomXboxController operator;

    private ControlBoard() {
        m_driver = new GenericHID(Ports.DRIVER_PORT);
        operator = new CustomXboxController(Ports.OPERATOR_PORT);
    }

    
    public void setOperatorRumble(boolean on) {
        operator.setRumble(on);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = getRightThrottle();
        double strafeAxis = getRightYaw();

        forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        SmartDashboard.putNumber("foreward Axis", forwardAxis);
        SmartDashboard.putNumber("strafe Axis", strafeAxis);

        if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

            double scaled_x = tAxes.x() - (deadband_vector.x()) / (1 - deadband_vector.x());
            double scaled_y = tAxes.y() - (deadband_vector.y()) / (1 - deadband_vector.y());
            SmartDashboard.putNumber("scaled x Axis", scaled_x);
            SmartDashboard.putNumber("scaled y Axis", scaled_y);
            return new Translation2d(scaled_x, scaled_y).scale(Constants.SwerveConstants.maxSpeed);
        }
        

    }

    public double getSwerveRotation() {
        double rotAxis = getLeftYaw();
        rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;
        SmartDashboard.putNumber("rot Axis", rotAxis);

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return Constants.SwerveConstants.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return m_driver.getRawButtonPressed(13);
    }
    
        public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

     public SwerveCardinal getSwerveSnap() {
        // CARDINAL SNAPS

        switch (operator.getController().getPOV()) {
            case kDpadUp:
                return SwerveCardinal.FORWARDS;
            case kDpadLeft:
                return SwerveCardinal.RIGHT;
            case kDpadRight:
                return SwerveCardinal.LEFT;
            case kDpadDown:
                return SwerveCardinal.BACKWARDS;
            default:
                return SwerveCardinal.NONE;
        }
            
    }


    // Align swerve drive with target
    public boolean getVisionAlign() {
        return operator.getButton(Button.RB);
    }


    // // Locks wheels in X formation
    public boolean getBrake() {
        return m_driver.getRawButton(4); //far left switch
    }

    // // Intake Controls
    // public boolean getIntake() {
    //     return operator.getTrigger(CustomXboxController.Side.RIGHT);
    // }

    // public boolean getReject() {
    //     return operator.getTrigger(CustomXboxController.Side.LEFT);
    // }
    
    
    //Returns positions from -1 to 1 
    private double getLeftYaw() {
        double leftYaw;
        if (m_driver.getRawAxis(Constants.leftXAxis) > Constants.ControllerConstants.ControllerLeftYawZero) {
            leftYaw = m_driver.getRawAxis(Constants.leftXAxis) / Constants.ControllerConstants.ControllerLeftYawHigh;
        } else
        leftYaw = m_driver.getRawAxis(Constants.leftXAxis) / Constants.ControllerConstants.ControllerLeftYawLow;

        if (m_driver.getRawAxis(Constants.leftXAxis) != 0){
            leftYaw = leftYaw - Constants.ControllerConstants.ControllerLeftYawZero;
        }
        SmartDashboard.putNumber("remote leftYaw", leftYaw);
        return 0;
    }

    //Returns positions from -1 to 1 
    private double getLeftThrottle() {
        double leftThrottle;
        if (m_driver.getRawAxis(Constants.leftYAxis) > Constants.ControllerConstants.ControllerLeftThrottleZero) {
            leftThrottle = m_driver.getRawAxis(Constants.leftYAxis) / Constants.ControllerConstants.ControllerLeftThrottleHigh;
        } else
        leftThrottle = m_driver.getRawAxis(Constants.leftYAxis) / Constants.ControllerConstants.ControllerLeftThrottleLow;

        if (m_driver.getRawAxis(Constants.leftYAxis) != 0){
            leftThrottle = leftThrottle - Constants.ControllerConstants.ControllerLeftThrottleZero;
        }
        //SmartDashboard.putNumber("remote XAxisLeft", leftThrottle);
        return leftThrottle;
    }

    private double getRightThrottle() {
        double rightThrottle;
        if (m_driver.getRawAxis(Constants.rightYAxis) > Constants.ControllerConstants.ControllerRightThrottleZero) {
            rightThrottle = m_driver.getRawAxis(Constants.rightYAxis) / Constants.ControllerConstants.ControllerRightThrottleHigh;
        } else
        rightThrottle = m_driver.getRawAxis(Constants.rightYAxis) / Constants.ControllerConstants.ControllerRightThrottleLow;

        if (m_driver.getRawAxis(Constants.rightYAxis) != 0){
            rightThrottle = rightThrottle - Constants.ControllerConstants.ControllerRightThrottleZero;
        }
        SmartDashboard.putNumber("remote rightThrottle", rightThrottle);
        return 0;
    }

    private double getRightYaw() {
        double rightYaw;
        if (m_driver.getRawAxis(Constants.rightXAxis) > Constants.ControllerConstants.ControllerRightYawZero) {
            rightYaw = m_driver.getRawAxis(Constants.rightXAxis) / Constants.ControllerConstants.ControllerRightYawHigh;
        } else
        rightYaw = m_driver.getRawAxis(Constants.rightXAxis) / Constants.ControllerConstants.ControllerRightYawLow;

        if (m_driver.getRawAxis(Constants.rightXAxis) != 0){
            rightYaw = rightYaw - Constants.ControllerConstants.ControllerRightYawZero;
        }
        SmartDashboard.putNumber("remote rightYaw", rightYaw);
        return 0;
    }
}

