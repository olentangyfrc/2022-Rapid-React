import frc.robot.subsystems.Climber.Climber;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;
import org.junit.*;

public class ClimberTest {
    Climber climber;
    public static final double DELTA = 1e-2; // acceptable deviation range
    @Before // this method will run before each test
    public void setup() {
    //assert HAL.initialize(500, 0);
    }

    @After // this method will run after each test
    public void shutdown() throws Exception {
    //intake.close();
    }

    @Test // marks this method as a test
    public void pushArmsForwardTest() {
        assertEquals(0.2, climber.getLeftLinearActuatorPercentOutput(), DELTA);
        assertEquals(0.2, climber.getRightLinearActuatorPercentOutput(), DELTA);
    }

    @Test
    public void retractTest() {
    //assertEquals(DoubleSolenoid.Value.kReverse, simPiston.get());
    }
}