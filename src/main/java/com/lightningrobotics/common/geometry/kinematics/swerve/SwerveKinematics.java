package com.lightningrobotics.common.geometry.kinematics.swerve;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveKinematics {

    private SwerveModuleState[] states;

    private DrivetrainSpeed speed;

    private double W;
    private double L;
    private double R;

    public SwerveKinematics(double width, double length) {
        this.W = width;
        this.L = length;
        this.R = Math.sqrt(W * W + L * L);
        this.states = new SwerveModuleState[DrivetrainConstants.NUM_MODULES];
    }

    public SwerveModuleState[] inverse(DrivetrainSpeed speed) {

        var FWD = speed.vx;
        var STR = speed.vy;
        var RCW = speed.omega;

        var A = STR - RCW * (L/R);
        var B = STR + RCW * (L/R);
        var C = FWD - RCW * (W/R);
        var D = FWD + RCW * (W/R);

        var FR_Speed = Math.sqrt(B*B + C*C);
        var FL_Speed = Math.sqrt(B*B + D*D);
        var RL_Speed = Math.sqrt(A*A + D*D);
        var RR_Speed = Math.sqrt(A*A + C*C);

        var FR_Angle = Math.atan2(B, C);
        var FL_Angle = Math.atan2(B, D);
        var RL_Angle = Math.atan2(A, D);
        var RR_Angle = Math.atan2(A, C);

        var MAX = UtilMath.max(FR_Speed, FL_Speed, RL_Speed, RR_Speed);

        if(MAX > DrivetrainConstants.MAX_SPEED) {
            FR_Speed /= MAX;
            FL_Speed /= MAX;
            RL_Speed /= MAX;
            RR_Speed /= MAX;
        }

        return new SwerveModuleState[]{
            new SwerveModuleState(FL_Speed, new Rotation2d(FL_Angle)),
            new SwerveModuleState(FR_Speed, new Rotation2d(FR_Angle)),
            new SwerveModuleState(RL_Speed, new Rotation2d(RL_Angle)),
            new SwerveModuleState(RR_Speed, new Rotation2d(RR_Angle))
        };

    }

    @Override
    public DrivetrainSpeed forward(SwerveModuleState[] states) {

        var FL = states[Drivetrain.Modules.FRONT_LEFT.getIdx()];
        var FR = states[Drivetrain.Modules.FRONT_RIGHT.getIdx()];
        var RL = states[Drivetrain.Modules.BACK_LEFT.getIdx()];
        var RR = states[Drivetrain.Modules.BACK_RIGHT.getIdx()];

        var BFL = FL.angle.getSin() * FL.velocity;
        var DFL = FL.angle.getCos() * FL.velocity;

        var BFR = FR.angle.getSin() * FR.velocity;
        var CFR = FR.angle.getCos() * FR.velocity;

        var ARL = RL.angle.getSin() * RL.velocity;
        var DRL = RL.angle.getCos() * RL.velocity;

        var ARR = RR.angle.getSin() * RR.velocity;
        var CRL = RR.angle.getCos() * RR.velocity;

        var A = (ARR + ARL) / 2d;
        var B = (BFL + BFR) / 2d;
        var C = (CFR + CRL) / 2d;
        var D = (DFL + DRL) / 2d;

        var ROT1 = (B - A) / L;
        var ROT2 = (C - D) / W;
        var ROT = (ROT1 + ROT2) / 2d;

        var FWD1 = ROT * (L / 2d) + A;
        var FWD2 = -ROT * (L / 2d) + B;
        var FWD = (FWD1 + FWD2) / 2d;

        var STR1 = ROT * (W / 2d) + C;
        var STR2 = -ROT * (W / 2d) + D;
        var STR = (STR1 + STR2) / 2d;

        var speed = new DrivetrainSpeed(FWD, STR, ROT);

        this.speed = speed;

        return speed;

    }

    public SwerveModuleState[] getStates() {
        return states;
    }

    public DrivetrainSpeed getSpeed() {
        return speed;
    }

}
