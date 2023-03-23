package com.team8013.frc2023.controlboard;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.controlboard.CustomXboxController.Axis;
import com.team8013.frc2023.controlboard.CustomXboxController.Button;
import com.team8013.frc2023.controlboard.CustomXboxController.Side;

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
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis : -strafeAxis;

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
            return Constants.SwerveConstants.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband))
                    / (1 - kSwerveDeadband);
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
        return operator.getButton(Button.R_JOYSTICK);
    }

    public boolean autoTest() {
        return operator.getButton(Button.L_JOYSTICK);
    }

    // // Locks wheels in X formation
    public boolean getBrake() {
        return m_driver.getRawButton(4); // far left switch
    }

    public boolean getArmDown() {
        return operator.getButton(Button.START);
    }

    public boolean getPickup() {
        return operator.getButton(Button.A);
    }

    public boolean getHybrid() {
        return operator.getButton(Button.B);
    }

    public boolean getMid() {
        return operator.getButton(Button.X);
    }

    public boolean getHigh() {
        return operator.getButton(Button.Y);
    }

    public boolean getZero() {
        return operator.getButton(Button.BACK);
    }

    public boolean getArmPullInToZero() {
        return operator.getButton(Button.RB);
    }

    public double getOperatorLeftThrottle() {
        return operator.getAxis(Side.LEFT, Axis.Y);
    }

    public double getOperatorLeftYaw() {
        return operator.getAxis(Side.LEFT, Axis.X);
    }

    public double getOperatorRightThrottle() {
        return operator.getAxis(Side.RIGHT, Axis.Y);
    }

    public double getOperatorRightYaw() {
        return operator.getAxis(Side.RIGHT, Axis.X);
    }

    public boolean getGrip() {
        return operator.getButton(Button.RB);
    }

    public boolean getRelease() {
        return operator.getButton(Button.LB);
    }

    public boolean getWantCone(){
        return (operator.getController().getPOV() == 90)||(m_driver.getRawButton(5)); //TODO: SET THIS
    }

    public boolean getWantCube(){
        return (operator.getController().getPOV() == 270)||(m_driver.getRawButton(6)); //TODO: SET THIS
    }

    public boolean getWantDoubleSubstation(){
        return operator.getButton(Button.LB) && operator.getButton(Button.RB) && operator.getTrigger(Side.LEFT) && operator.getTrigger(Side.RIGHT);
    }

    // // Intake Controls
    // public boolean getIntake() {
    // return operator.getTrigger(CustomXboxController.Side.RIGHT);
    // }

    // public boolean getReject() {
    // return operator.getTrigger(CustomXboxController.Side.LEFT);
    // }

    // Returns positions from -1 to 1
    private double getLeftYaw() {
        double leftYaw = m_driver.getRawAxis(Constants.leftXAxis);

        if (leftYaw != 0) {
            leftYaw = leftYaw - Constants.ControllerConstants.ControllerLeftYawZero;
        }

        if (leftYaw > kSwerveDeadband) {
            leftYaw = (leftYaw / (Constants.ControllerConstants.ControllerLeftYawHigh
                    + (Constants.ControllerConstants.isControllerOne
                            ? -Constants.ControllerConstants.ControllerLeftYawZero
                            : Constants.ControllerConstants.ControllerLeftYawZero)));
        } else if (leftYaw < -kSwerveDeadband) {
            leftYaw = (leftYaw / (Constants.ControllerConstants.ControllerLeftYawLow
                    + Constants.ControllerConstants.ControllerLeftYawZero));
        }

        if (leftYaw > 1) {
            leftYaw = 1;
        }

        if (leftYaw < -1) {
            leftYaw = -1;
        }

        // SmartDashboard.putNumber("remote leftYaw", leftYaw);
        return leftYaw;
    }

    // Returns positions from -1 to 1
    // private double getLeftThrottle() {
    // double leftThrottle = m_driver.getRawAxis(Constants.leftYAxis);

    // if (leftThrottle != 0){
    // leftThrottle = leftThrottle -
    // Constants.ControllerConstants.ControllerLeftThrottleZero;
    // }

    // if (leftThrottle > kSwerveDeadband){
    // leftThrottle = (leftThrottle /
    // (Constants.ControllerConstants.ControllerLeftThrottleHigh +
    // Constants.ControllerConstants.ControllerLeftThrottleZero));
    // }
    // else if (leftThrottle < -kSwerveDeadband){
    // leftThrottle = (leftThrottle /
    // (Constants.ControllerConstants.ControllerLeftThrottleLow +
    // Constants.ControllerConstants.ControllerLeftThrottleZero));
    // }

    // if (leftThrottle>1){
    // leftThrottle = 1;
    // }

    // if (leftThrottle<-1){
    // leftThrottle = -1;
    // }

    // //SmartDashboard.putNumber("remote leftThrottle", leftThrottle);
    // return leftThrottle;
    // }

    private double getRightThrottle() {
        double rightThrottle = m_driver.getRawAxis(Constants.rightYAxis);

        if (rightThrottle != 0) {
            rightThrottle = rightThrottle - Constants.ControllerConstants.ControllerRightThrottleZero;
        }

        if (rightThrottle > (Constants.ControllerConstants.isControllerOne ? kSwerveDeadband : 0.102)) {
            rightThrottle = (rightThrottle / (Constants.ControllerConstants.ControllerRightThrottleHigh
                    + (Constants.ControllerConstants.isControllerOne
                            ? -Constants.ControllerConstants.ControllerRightThrottleZero
                            : Constants.ControllerConstants.ControllerRightThrottleZero)));
        } else if (rightThrottle < -kSwerveDeadband) {
            rightThrottle = (rightThrottle / (Constants.ControllerConstants.ControllerRightThrottleLow
                    + Constants.ControllerConstants.ControllerRightThrottleZero));
        }

        if (rightThrottle > 1) {
            rightThrottle = 1;
        }

        if (rightThrottle < -1) {
            rightThrottle = -1;
        }

        // SmartDashboard.putNumber("remote rightThrottle", rightThrottle);
        return rightThrottle;
    }

    private double getRightYaw() {
        double rightYaw = m_driver.getRawAxis(Constants.rightXAxis);

        if (rightYaw != 0) {
            rightYaw = rightYaw - Constants.ControllerConstants.ControllerRightYawZero;
        }

        if (rightYaw > kSwerveDeadband) {
            rightYaw = (rightYaw / (Constants.ControllerConstants.ControllerRightYawHigh
                    + -Constants.ControllerConstants.ControllerRightYawZero));
        } else if (rightYaw < -kSwerveDeadband) {
            rightYaw = (rightYaw / (Constants.ControllerConstants.ControllerRightYawLow
                    + Constants.ControllerConstants.ControllerRightYawZero));
        }

        if (rightYaw > 1) {
            rightYaw = 1;
        }

        if (rightYaw < -1) {
            rightYaw = -1;
        }

        // SmartDashboard.putNumber("remote rightYaw", rightYaw);
        return rightYaw;
    }
}
