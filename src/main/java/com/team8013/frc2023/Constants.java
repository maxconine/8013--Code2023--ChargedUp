package com.team8013.frc2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.lib.util.SwerveModuleConstants;
import com.team254.lib.geometry.Rotation2d;
import com.team8013.frc2023.subsystems.Limelight.LimelightConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

public class Constants {

        // toggle constants between comp bot and practice bot ("epsilon")
        public static final boolean isComp = true;

        // robot loop time
        public static final double kLooperDt = 0.02;

        /* Control Board */
        public static final double kTriggerThreshold = 0.2;

        public static final double stickDeadband = 0.02;
        public static final int leftXAxis = 0;
        public static final int leftYAxis = 1;
        public static final int rightXAxis = 3;
        public static final int rightYAxis = 4;

        public static final class SwerveConstants {
                public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(20.25);
                public static final double wheelBase = Units.inchesToMeters(20.25);

                public static final double wheelDiameter = Units.inchesToMeters(4.0);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                // This is for the comp bot
                public static final double driveGearRatio = 6.75; // flipped gear ratio
                                                                  // https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
                public static final double angleGearRatio = 15.43; // 8:32:24--14:72 = 15.43 ratio

                /* Practice Bot */
                // public static final double driveGearRatio = 6.525; //6.55
                // public static final double angleGearRatio = 10.29; // 72:14:24:12

                // 43.75 - number to divide driven distance by

                public static final Translation2d m_frontLeftLocation = new Translation2d(wheelBase / 2.0,
                                trackWidth / 2.0);
                public static final Translation2d m_frontRightLocation = new Translation2d(wheelBase / 2.0,
                                -trackWidth / 2.0);
                public static final Translation2d m_backLeftLocation = new Translation2d(-wheelBase / 2.0,
                                trackWidth / 2.0);
                public static final Translation2d m_backRightLocation = new Translation2d(-wheelBase / 2.0,
                                -trackWidth / 2.0);

                public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocations = {
                                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation };

                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                swerveModuleLocations);

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 25;
                public static final int anglePeakCurrentLimit = 40;
                public static final double anglePeakCurrentDuration = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveContinuousCurrentLimit = 35;
                public static final int drivePeakCurrentLimit = 60;
                public static final double drivePeakCurrentDuration = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /* Angle Motor PID Values */
                public static final double angleKP = 0.3;
                public static final double angleKI = 0.0;
                public static final double angleKD = 0.0;
                public static final double angleKF = 0.0;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.05; // 0.05
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                /* Drive Motor Characterization Values */
                public static final double driveKS = (0.32 / 12);
                public static final double driveKV = (1.51 / 12);
                public static final double driveKA = (0.27 / 12);

                /* Swerve Profiling Values */ 
                public static final double maxSpeed = 5.02; // meters per second MAX : 5.02 m/s Linear Speed (meters /sec) = motor speed (RPM) / gear ratio * pi * wheel diameter (meters) / 60
                // LS = 6380/6.75*pi*0.1016/60 = 5.028
                public static final double maxAngularVelocity = 8.0;

                /* Neutral Modes */
                public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
                public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

                /* Motor Inverts */
                public static final boolean driveMotorInvert = false;
                public static final boolean angleMotorInvert = true;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;

                /* Controller Invert */
                public static final boolean invertYAxis = false;
                public static final boolean invertRAxis = true;
                public static final boolean invertXAxis = true;

                /*** MODULE SPECIFIC CONSTANTS ***/

                /* Front Left Module - Module 0 */ // BACK LEFT
                public static final class Mod0 {
                        public static final double compAngleOffset = 339.86;// 2.7;

                        public static SwerveModuleConstants SwerveModuleConstants() {
                                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                                                compAngleOffset);
                        }
                }

                /* Front Right Module - Module 1 */ // BACK RIGHT
                public static final class Mod1 {
                        public static final double compAngleOffset = 1.1;// 346;

                        public static SwerveModuleConstants SwerveModuleConstants() {
                                return new SwerveModuleConstants(Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER,
                                                compAngleOffset);
                        }
                }

                /* Back Left Module - Module 2 */ // front left
                public static final class Mod2 {
                        public static final double compAngleOffset = 30.93;// 100;

                        public static SwerveModuleConstants SwerveModuleConstants() {
                                return new SwerveModuleConstants(Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER,
                                                compAngleOffset);
                        }
                }

                /* Back Right Module - Module 3 */ // front right on practice bot
                public static final class Mod3 {
                        public static final double compAngleOffset = 30.22; // 129.5;

                        public static SwerveModuleConstants SwerveModuleConstants() {
                                return new SwerveModuleConstants(Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER,
                                                compAngleOffset);
                        }
                }
        }

        public static final class SnapConstants {
                public static final double kP = 5.95; // og 5.0 //6.0 seems to work with 0.15 kD
                public static final double kI = 0.0; // og 0
                public static final double kD = 0.15; // og 0
                public static final double kTimeout = 0.25;
                public static final double kEpsilon = 5.0; // og 1.0

                // Constraints for the profiled angle controller
                public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI; // og 2.0 * pi
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond,2);

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }

        public static final class VisionAlignConstants {
                public static final double kP = 6.37;
                public static final double kI = 0.0;
                public static final double kD = 0.10;
                public static final double kTimeout = 0.25;
                public static final double kEpsilon = 5.0;

                // Constraints for the profiled angle controller
                public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10.0 * Math.PI;

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }

        public static final class AutoConstants {
                public static final double kSlowSpeedMetersPerSecond = 2.2; // 1.7
                public static final double kSlowAccelerationMetersPerSecondSquared = 2.3; // 2.0

                public static final double kMaxSpeedMetersPerSecond = 2.2; // og 2.2
                public static final double kMaxAccelerationMetersPerSecondSquared = 2.3; // og 2.3

                public static final double kSlowMaxAngularSpeedRadiansPerSecond = 0.4 * Math.PI; // og 0.8
                public static final double kSlowMaxAngularSpeedRadiansPerSecondSquared = Math
                                .pow(kSlowMaxAngularSpeedRadiansPerSecond, 2);

                public static final double kMaxAngularSpeedRadiansPerSecond = 0.8 * Math.PI; // 0g
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(
                                kMaxAngularSpeedRadiansPerSecond,
                                2);

                public static final double kPXController = 1; // og 1 0.2 works
                public static final double kPYController = 1; // og 1 0.3 works
                public static final double kPThetaController = 3.8; // 3.8; // og 5

                // Constraint for the motion profilied robot angle controller
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

                // Constraint for the motion profilied robot angle controller
                public static final TrapezoidProfile.Constraints kSlowThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kSlowMaxAngularSpeedRadiansPerSecond, kSlowMaxAngularSpeedRadiansPerSecondSquared);

                public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed,
                                double endSpeed) {
                        TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
                        config.setKinematics(Constants.SwerveConstants.swerveKinematics);
                        config.setStartVelocity(startSpeed);
                        config.setEndVelocity(endSpeed);
                        config.addConstraint(new CentripetalAccelerationConstraint(3.0));
                        return config;
                }

                // Trajectory Speed Configs
                public static final TrajectoryConfig defaultSpeedConfig = new TrajectoryConfig(
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.SwerveConstants.swerveKinematics);

                public static final TrajectoryConfig slowSpeedConfig = new TrajectoryConfig(
                                kSlowSpeedMetersPerSecond,
                                kSlowAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.SwerveConstants.swerveKinematics)
                                .setStartVelocity(0)
                                .setEndVelocity(0);

                /* TIME CONSTANTS */
                public static final double firstDropHighWait = 5; //wait 5 seconds for the cone to be dropped
                public static final double pickupPieceWait = 2;

                /*BALANCE CONSTANTS*/
                public static final double balance_kP = .2;
                public static final double balance_kI = 0;
                public static final double balance_kD = 0;
                public static final double balance_kMinOutput = -0.8;
                public static final double balance_kMaxOutput = 0.8;
                public static final double balance_PositionTolerance = 1;

                //non pid
                public static final double firstAngle = 5; //within +- degrees, turn around (11 to 15 degrees irl)
                public static final double secondAngle = 5; //within +- degrees, then stop
                public static final double firstSpeed = 1.0; // meters per second
                public static final double secondSpeed = 0.4; // meters per second
        }

        public static final class VisionConstants {
                public static final LimelightConstants kLimelightConstants = new LimelightConstants();
                static {
                        kLimelightConstants.kName = "Limelight";
                        kLimelightConstants.kTableName = "limelight";
                        kLimelightConstants.kHeight = 0.79; // meters
                        kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
                }

                public static final double kHorizontalFOV = 59.6; // degrees
                public static final double kVerticalFOV = 49.7; // degrees
                public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

                // lookahead time
                public static final double kLookaheadTime = 0.0; // 1.10 as latest

                /* Goal Tracker Constants */
                public static final double kMaxTrackerDistance = 8.0;
                public static final double kMaxGoalTrackAge = 10.0;
                public static final double kMaxGoalTrackSmoothingTime = 1.5;
                public static final double kCameraFrameRate = 90.0;

                public static final double kTrackStabilityWeight = 0.0;
                public static final double kTrackAgeWeight = 10.0;
                public static final double kTrackSwitchingWeight = 100.0;

                // public static final int kDefaultPipeline = 0;
                public static final double kGoalHeight = 2.63; // meters
                public static final double kGoalRadius = Units.inchesToMeters(.5); // meters

                public static final double fieldLength = Units.inchesToMeters(651.25);
                public static final double fieldWidth = Units.inchesToMeters(315.5);
        }

        /*** SUBSYSTEM CONSTANTS ***/

        public static final class IntakeConstants {

                public static final double kSingulatorVelocityConversion = (600.0 / 2048.0) * (1.0 / 1.9);

                public static final double kSingulatorP = 0.07;
                public static final double kSingulatorI = 0.0;
                public static final double kSingulatorD = 0.01;
                public static final double kSingulatorF = 0.045;

                public static final double kIntakingVoltage = 10;
                public static final double kSpittingVoltage = -8;
                public static final double kRejectingVoltage = -5;

                public static final double kSingulatorVelocity = 2300.0;

                public static final double kDeployVoltage = 4.0;
                public static final double kInHoldingVoltage = 1.2;
                public static final double kOutHoldingVoltage = 1.5;

                public static final double kDeployCurrentLimit = 60;

                public static final double kIntakeRejectTime = 1.0;
                public static final double kSingulatorReverseDelay = 0.5;
        }

        public static final class ArmConstants {
                public static final double kStatorCurrentLimit = 25; // 80.0;
                public static final double kZeroCurrentLimit = 50; // 80.0;
                public static final double kTriggerThresholdCurrent = 60;

                // arm constants
                public static final int changeArmManualAmount = 2000;

                public static final int kMinHeight = 0; // ticks
                // public static final int kMaxHeight = 244984; // ticks
                // public static final int kTravelDistance = kMaxHeight - kMinHeight + 500; //
                // ticks

                // TODO I have no idea the max height ticks
                public static final int kPickupTravelDistance = 144000;
                public static final int kHybridTravelDistance = 100000; // kLeftTravelDistance * 0.75
                public static final int kMidTravelDistance = 120000; // kLeftTravelDistance * 0.75
                public static final int kHighTravelDistance = 270000; // kLeftTravelDistance * 0.75
                public static final int kAutoHighTravelDistance = 290000; // kLeftTravelDistance * 0.75
                public static final int kDoubleSubstationTravelDistance = 100000;

                /* GENERAL CLIMBER CONSTANTS USED */

                public static final double kBarContactAngleEpsilon = 2.0;

                public static final int kSafetyMinimum = -7000; // minimum outside 0 ticks

                public static final double kTravelDistanceEpsilon = 20000;

                public static final double canDropConeDistance = 5000;// within _ motor rotations*2048, start to drop

        }

        public static final class PivotConstants {
                public static final int kStatorCurrentLimit = 30; // 80 Amps for Neo motor
                public static final double kTriggerThresholdCurrent = 60;

                public static final boolean canCoderInvert = true; // if false(default) is ccw+ when observing from the led side
                public static final double canCoderOffset = 0; //program subtracts this value to the cancoder angle to make it 0

                public static final double neo_kP = 0.5;
                public static final double neo_kI = 0.0;
                public static final double neo_kD = 0.0;
                public static final double neo_kIz = 0;
                public static final double neo_kFF = 0;
                public static final double kMaxOutput = 1;
                public static final double kMinOutput = -1;

                public static final double kP = 0.5;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kGravity = 0.0; //percent output required to hold the arm horisontal

                // gear ratio for one full rotation of the pivot
                // (3*3*3)* (227/14) = 437.785714286 used to be 972.857 //TODO:Double check this
                public static final double oneDegreeOfroation = 1.21607142857; // used to be 2.702381;

                // 14 tooth to 226 tooth
                // rotations*360 to get degrees

                // pivot constants
                public static final double degreesOffToReset = 3; //degrees the motor is off by
                public static final double ticksOffToReset = degreesOffToReset*oneDegreeOfroation*2048; //ticks the motor is off by

                public static final double degreesCanExtendArm = 4;

                // pivot degree constants
                public static final double kPickupTravelDistance = 40 + 2;
                public static final double kHybridTravelDistance = 47.37 + 5; // degrees
                public static final double kMidTravelDistance = 95.47 + 5;
                public static final double kHighTravelDistance = 106.2 + 5;
                public static final double kAutoHighTravelDistance = 105 + 5;
                public static final double kDoubleSubstationTravelDistance = 130;

        }

        public static final class ClawConstants {
                /* PIVOT */

                public static final double piv_kP = .0085;
                public static final double piv_kI = 0;
                public static final double piv_kD = 0.0004;

                public static final double piv_kMaxOutput = 0.95;
                public static final double piv_kMinOutput = -0.95;

                public static final double piv_MaxRotation = 450; // how many degrees in either direction it can spin
                public static final double piv_MinRotation = -450;

                public static final double piv_ZeroRotation = -10;
                public static final double piv_90Rotation = 87;
                public static final double piv_180Rotation = 180;



                public static final double pivotGearRatio = 1.535714; // (86 / 56);

                public static final double kPivotMinDistance = -pivotGearRatio; // encoder hard limit one full rotation
                                                                                // either side
                public static final double kPivotMaxDistance = pivotGearRatio; // encoder hard limit


                /*CAN CODER */
                public static final double canCoderOffset = 208.38; //program subtracts this value to the cancoder angle to make it 0
                public static final boolean canCoderInvert = false; //program subtracts this value to the cancoder angle to make it 0

                /* CLAW */

                public static final double grip_kP = 0.35;
                public static final double grip_kI = 0;
                public static final double grip_kD = 0.001;

                // how fast you can manually control claw (percent output)
                public static final double grip_kMaxOutput = 0.95;
                public static final double grip_kMinOutput = -0.95;

                public static final double kClawOpenDistance = 0; // position for pickup
                public static final double kClawMinDistance = -10; // encoder hard limit
                public static final double kClawMaxDistance = 10; // encoder hard limit

                public static final double grip_rateDiff = 0.97; // Percent of max the speed needs to decrease from

                public static final double gripGearRatio = 42; // unsure if this value is correct
        }

        public static final class ControllerConstants {
                public static final boolean isControllerOne = true;

                // Controller 1 left side:
                public static final double ControllerOneLeftThrottleZero = -0.125;
                public static final double ControllerOneLeftYawZero = 0.039370;

                public static final double ControllerOneLeftThrottleHigh = 0.787402;
                public static final double ControllerOneLeftThrottleLow = 0.968750;

                public static final double ControllerOneLeftYawHigh = 0.86612;
                public static final double ControllerOneLeftYawLow = 0.77338;

                // Controller 1 right side:
                public static final double ControllerOneRightThrottleZero = 0.055118;
                public static final double ControllerOneRightYawZero = 0.055118;

                public static final double ControllerOneRightYawHigh = 0.866142;
                public static final double ControllerOneRightYawLow = 0.765625;

                public static final double ControllerOneRightThrottleHigh = 0.732283;
                public static final double ControllerOneRightThrottleLow = 0.601563;

                // Controller 2 left side:
                public static final double ControllerTwoLeftThrottleZero = -0.023438;
                public static final double ControllerTwoLeftYawZero = -0.078125;

                public static final double ControllerTwoLeftThrottleHigh = 0.834646;
                public static final double ControllerTwoLeftThrottleLow = 0.867188;

                public static final double ControllerTwoLeftYawHigh = 0.748031;
                public static final double ControllerTwoLeftYawLow = 0.890625;

                // Controller 2 right side:
                public static final double ControllerTwoRightThrottleZero = -0.054688; // high 0.007874
                public static final double ControllerTwoRightYawZero = 0.062992;

                public static final double ControllerTwoRightYawHigh = 0.866142;
                public static final double ControllerTwoRightYawLow = 0.664063;

                public static final double ControllerTwoRightThrottleHigh = 0.669291;
                public static final double ControllerTwoRightThrottleLow = 0.664063;

                // Controller left side:
                public static final double ControllerLeftThrottleZero = isControllerOne ? ControllerOneLeftThrottleZero
                                : ControllerTwoLeftThrottleZero;
                public static final double ControllerLeftYawZero = isControllerOne ? ControllerOneLeftYawZero
                                : ControllerTwoLeftYawZero;

                public static final double ControllerLeftThrottleHigh = isControllerOne ? ControllerOneLeftThrottleHigh
                                : ControllerTwoLeftThrottleHigh;
                public static final double ControllerLeftThrottleLow = isControllerOne ? ControllerOneLeftThrottleLow
                                : ControllerTwoLeftThrottleLow;

                public static final double ControllerLeftYawHigh = isControllerOne ? ControllerOneLeftYawHigh
                                : ControllerTwoLeftYawHigh;
                public static final double ControllerLeftYawLow = isControllerOne ? ControllerOneLeftYawLow
                                : ControllerTwoLeftYawLow;

                // Controller right side:
                public static final double ControllerRightThrottleZero = isControllerOne
                                ? ControllerOneRightThrottleZero
                                : ControllerTwoRightThrottleZero;
                public static final double ControllerRightYawZero = isControllerOne ? ControllerOneRightYawZero
                                : ControllerTwoRightYawZero;

                public static final double ControllerRightYawHigh = isControllerOne ? ControllerOneRightYawHigh
                                : ControllerTwoRightYawHigh;
                public static final double ControllerRightYawLow = isControllerOne ? ControllerOneRightYawLow
                                : ControllerTwoRightYawLow;

                public static final double ControllerRightThrottleHigh = isControllerOne
                                ? ControllerOneRightThrottleHigh
                                : ControllerTwoRightThrottleHigh;
                public static final double ControllerRightThrottleLow = isControllerOne ? ControllerOneRightThrottleLow
                                : ControllerTwoRightThrottleLow;

        }

        // Timeout constants
        public static final int kLongCANTimeoutMs = 100;
        public static final int kCANTimeoutMs = 10;
}
