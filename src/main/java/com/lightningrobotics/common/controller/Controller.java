package com.lightningrobotics.common.controller;

/**
 * Represents an abstract closed loop controller. 
 * Base for PID controller and other variants.
 */
public abstract class Controller {

    public abstract void enableContinuousInput(double minimumInput, double maximumInput);

    public abstract double calculate(double measurement, double setpoint);

}
