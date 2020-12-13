package frc.lightning.util;

import org.junit.Test;
import static org.junit.Assert.assertEquals;

public class LightningMathTest {
    
    @Test
    public void limitBoundsTest() {
        double testValue = 5d;
        double testLow       = LightningMath.limit(testValue, 10d, 15d); // 10?
        double testMiddle    = LightningMath.limit(testValue, 0d, 10d); // 5?
        double testHigh      = LightningMath.limit(testValue, -5d, 0d); // 0?
        double testLowBound  = LightningMath.limit(0d, 0d, 10d); // 0?
        double testHighBound = LightningMath.limit(10, 0d, 10d); // 10?
        assertEquals(testLow, 10d,       Math.ulp(1d));
        assertEquals(testMiddle, 5d,     Math.ulp(1d));
        assertEquals(testHigh, 0d,       Math.ulp(1d)); 
        assertEquals(testLowBound, 0d,   Math.ulp(1d)); 
        assertEquals(testHighBound, 10d, Math.ulp(1d));        
    }

}
