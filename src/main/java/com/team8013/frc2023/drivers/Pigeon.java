package com.team8013.frc2023.drivers;

import com.ctre.phoenix.sensors.Pigeon2;
import com.team8013.frc2023.Ports;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon {

    private static Pigeon mInstance;

    public static Pigeon getInstance() {
        if (mInstance == null) {
            mInstance = new Pigeon(Ports.PIGEON);
        }
        return mInstance;
    }

    // Actual pigeon object
    private final Pigeon2 mGyro;

    // Configs

    private Pigeon(int port) {        
        mGyro = new Pigeon2(port, "canivore1");
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(mGyro.getRoll());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(mGyro.getPitch());
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setYaw(double angleDeg) {
        mGyro.setYaw(angleDeg);
    }



}