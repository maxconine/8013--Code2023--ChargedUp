package com.team8013.frc2023;

public class Ports {

    /*** Operator ports ***/
    public static final int DRIVER_PORT = 2;
    public static final int OPERATOR_PORT = 1;

    public static final int CLAW_PIV = 24;
    public static final int CLAW_GRIP = 23;

    public static final int CLAW_CANCODER = 26;

    public static final int CLAW_PIV_ENCODER_A = 0;
    public static final int CLAW_PIV_ENCODER_B = 1;

    public static final int CLAW_GRIP_ENCODER_A = 2;
    public static final int CLAW_GRIP_ENCODER_B = 3;

    public static final int PIVOT = 16;
    public static final int ARM = 17;

    // public static final int INTAKE_DEPLOY = 18;
    // public static final int INTAKE_ROLLER_TOP = 19;
    // public static final int INTAKE_ROLLER_BOTTOM = 19;

    public static final int CANDLE = 20;
    public static final int PIV_CANCODER = 21;

    public static final int PDP = 13;

    /*** SWERVE MODULE PORTS ***/

    /*
     * Swerve Modules go:
     * 1 2
     * 3 4
     */

    public static final int FL_DRIVE = 5;
    public static final int FL_ROTATION = 6;
    public static final int FL_CANCODER = 1;

    public static final int FR_DRIVE = 7;
    public static final int FR_ROTATION = 8;
    public static final int FR_CANCODER = 2;

    public static final int BL_DRIVE = 9;
    public static final int BL_ROTATION = 10;
    public static final int BL_CANCODER = 3;

    public static final int BR_DRIVE = 11;
    public static final int BR_ROTATION = 12;
    public static final int BR_CANCODER = 4;

    public static final int PIGEON = 13;

}
