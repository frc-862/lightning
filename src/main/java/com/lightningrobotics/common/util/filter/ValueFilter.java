package com.lightningrobotics.common.util.filter;

public interface ValueFilter {

    void reset();

    double filter(double value);

    double get();
    
}