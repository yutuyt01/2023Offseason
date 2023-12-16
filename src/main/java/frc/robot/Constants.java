package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.67;
        public static final double kTurningMotorGearRatio = 1 / 45.0;
        // Converting revolution of motor to position moved on the ground - Multiply gear ratio, pi, and diameter to get distance around wheel.
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        // Converting revolution to radians - 2 pi radians per revolution.
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        // Converting pulses to radians. 7 was the attempt to add pulses per revolution to the calculation. 
        // The 1.31755... is the magic number.
        public static final double kTurningEncoderPPRad = kTurningEncoderRot2Rad / 7 / 1.317552505982753;
        //public static final double kTurningEncoderPPRad = kTurningEncoderRot2Rad / 7 / 1.2;
        // Drive RPM to m/s - since we have a conversion from revolutions to meters already, all we need to do is handle minutes -> seconds.
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        // Similar situation for turning RPM to rad/s. 
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        
        public static final double kPTurning = 1.5;
        public static final double kPTurningFL = 1.5;
    }

    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(20.375);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 23;
        public static final int kBackLeftDriveMotorPort = 27;
        public static final int kFrontRightDriveMotorPort = 28;
        public static final int kBackRightDriveMotorPort = 24;

        public static final int kFrontLeftTurningMotorPort = 20;
        public static final int kBackLeftTurningMotorPort = 29;
        public static final int kFrontRightTurningMotorPort = 31;
        public static final int kBackRightTurningMotorPort = 22;

        public static final int[] kFrontLeftTurningEncoderPorts = new int[] {2, 3};
        public static final int[] kBackLeftTurningEncoderPorts = new int[] {4, 5};
        public static final int[] kFrontRightTurningEncoderPorts = new int[] {0, 1};
        public static final int[] kBackRightTurningEncoderPorts = new int[] {6, 7};

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 4.06;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 5.17;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.85;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.09;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2.75;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowModeSpeedMultiplier = 0.25;
        public static final double kCubePickupSpeedMultiplier = 0.6;

        public static final double kEncoderCountsPerRev = 9999;
        
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2.75;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 4;
        public static final double kPYController = 3;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kSwivelControllerPort = 1;
        public static final int kTopHalfButtonBoardPort = 2;
        public static final int kBottomHalfButtonBoardPort = 3;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 2;
        public static final int kDriverSlowMode = 10;

        public static final int kSwivelYAxis = 1;

        public static final double kDeadband = 0.05;
        public static final double kZDeadband = 0.07;

        public static final double kSwivelDeadband = 0.3;
        public static final double kSwivelMaxPercentSpeed = 20;
        
		

        
    }
}
