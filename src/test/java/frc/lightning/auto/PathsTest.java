package frc.lightning.auto;

import org.junit.Test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lightning.auto.Autonomous;
import frc.lightning.auto.Paths;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertNull;

public class PathsTest {

    @Test
    public void empty() {
        assertEquals((double) frc.lightning.auto.Paths.getPaths().size(), 0d, Math.ulp(1d));
    }

    @Test
    public void addRemove() {
        frc.lightning.auto.Paths.register("TEST", new frc.lightning.auto.Path("Name", null));
        assertEquals(frc.lightning.auto.Paths.getPath("TEST").getName(), "Name");
        assertEquals((double) frc.lightning.auto.Paths.getPaths().size(), 1d, Math.ulp(1d));
        frc.lightning.auto.Paths.removePath("TEST");
        assertEquals((double) frc.lightning.auto.Paths.getPaths().size(), 0d, Math.ulp(1d));
    }
    
}
