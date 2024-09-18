package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreaks {
    private static final DigitalInput input1 = new DigitalInput(1);
	private static final DigitalInput input0 = new DigitalInput(0);
    public boolean m_input0;
    public boolean m_input1;
    // public BeamBreaks() {
    //     m_input0 = false;
    //     m_input1 = false;
    // }

    public void updateBeamBreaks () {
        m_input0 = input0.get();
        m_input1 = input1.get();
        SmartDashboard.putBoolean("input0", m_input0);
        SmartDashboard.putBoolean("input1", m_input1);
    }

}
