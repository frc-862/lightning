package com.lightningrobotics.common.controller;

import com.lightningrobotics.common.util.*;

/**
 * Simple PID controller with simple Feedforward coefficient.
 */
public class PIDFController extends Controller {

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double period;

    private boolean continuous = false;
    private double maximumInput;
    private double minimumInput;
    private double maximumIntegral = 1.0;
    private double minimumIntegral = -1.0;

    private double error;
    private double prevError;
    private double acculumatedError;
    private double direction;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 0.02);
    }

    public PIDFController(double kP, double kI, double kD, double kF, double period) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.period = period;

        reset();
    }
    
    @Override
    public double calculate(double measurement, double setpoint) {

        // Note previous error
        prevError = error;
        
        // Calculate error
        if (continuous) {
            var errorBound = (maximumInput - minimumInput) / 2.0;
            error = LightningMath.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            error = setpoint - measurement;
        }

        // Calculate direction for feedforward gain
        direction = Math.signum(error);

        // Calculate derivative
        var derivative = (error - prevError) / period;

        // Calculate integral
        if(kI != 0d) 
            acculumatedError = LightningMath.clamp(acculumatedError + (error * period), minimumIntegral / kI, maximumIntegral / kI);
        var integral = acculumatedError;

        // Calculate output from error and gains
        return (kP * error) + (kD * derivative) + (kI * integral) + (kF * direction);
    
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    public void reset() {
        error = 0;
        prevError = 0;
        acculumatedError = 0;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

}

