import frc.robot.subsystems.Elevator.Elevator;

import static org.junit.Assert.*;
import org.junit.*;

public class ElevatorTest {
    @Before // this method will run before each test
    public void setup() {
    //assert HAL.initialize(500, 0);
    }

    @After // this method will run after each test
    public void shutdown() throws Exception {
    //intake.close();
    }

    @Test // marks this method as a test
    public void doesntWorkWhenClosed() {
    //assertEquals(0.0, simMotor.getSpeed(), DELTA);
    }

    @Test
    public void retractTest() {
    //assertEquals(DoubleSolenoid.Value.kReverse, simPiston.get());
    }
}