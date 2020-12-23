/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lightning;

/**
 * Lightning config base class. Houses constants for a robot.
 */
public abstract class LightningConfig {

    public static final boolean TUNING_ENABLED = true;
    
    public static final double VOLT_LIMIT = 12d;

    public abstract double getTicsPerRev();
    
    public abstract double getMaxRPM();

    public abstract double getWheelDiameterInches();

    public abstract double getGearReduction();

    public double getWheelCircumferenceInches() {
        return getWheelDiameterInches() * Math.PI;
    }

}
