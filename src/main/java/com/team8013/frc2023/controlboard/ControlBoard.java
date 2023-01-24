package com.team8013.frc2023.controlboard;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.controlboard.CustomXboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;

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

    private final GenericHID m_driver;;
    public final CustomXboxController operator;

    private ControlBoard() {
        m_driver = new GenericHID(Constants.kDriveControllerPort);;
        operator = new CustomXboxController(Constants.kOperatorControllerPort);
    }

    
    public void setOperatorRumble(boolean on) {
        operator.setRumble(on);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = getYAxisRight();
        double strafeAxis = getXAxisRight();

        forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

            double scaled_x = tAxes.x() - (deadband_vector.x()) / (1 - deadband_vector.x());
            double scaled_y = tAxes.y() - (deadband_vector.y()) / (1 - deadband_vector.y());
            return new Translation2d(scaled_x, scaled_y).scale(Constants.SwerveConstants.maxSpeed);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = getXAxisLeft();
        rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;

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
    

    private double getXAxisLeft() { //beutiful precision
        double XAxisLeft;
        if (m_driver.getRawAxis(Constants.leftXAxis) > 0) {
            XAxisLeft = m_driver.getRawAxis(Constants.leftXAxis) / 0.826;
        } else
        XAxisLeft = m_driver.getRawAxis(Constants.leftXAxis) / 0.807;

        XAxisLeft = XAxisLeft - 0.03937007;
        //SmartDashboard.putNumber("remote XAxisLeft", XAxisLeft);
        return XAxisLeft;

    }

    private double getYAxisRight() {
        double YAxisRight;
        YAxisRight = m_driver.getRawAxis(Constants.rightYAxis);
        if (m_driver.getRawAxis(Constants.rightYAxis) > 0) {
             YAxisRight = m_driver.getRawAxis(Constants.rightYAxis) / 0.7052;
         } else { // negative value
           YAxisRight = m_driver.getRawAxis(Constants.rightYAxis) / 0.6449;
         }

        YAxisRight = YAxisRight - 0.047244; 
        //SmartDashboard.putNumber("remote YAxisRight", YAxisRight);
        return YAxisRight;
    }

    private double getXAxisRight() {
        double XAxisRight;
        XAxisRight = m_driver.getRawAxis(Constants.rightXAxis);
         if (m_driver.getRawAxis(Constants.rightXAxis) > 0) {
             XAxisRight = m_driver.getRawAxis(Constants.rightXAxis) / 0.8443;
         } else {
             XAxisRight = m_driver.getRawAxis(Constants.rightXAxis) / 0.8193;
         }
        
        XAxisRight = XAxisRight - 0.055118;

        //SmartDashboard.putNumber("remote XAxisRight", XAxisRight);
        return XAxisRight;
    }
}

