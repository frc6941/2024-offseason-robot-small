package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class led extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_Buffer;

    public led() {
        m_led = new AddressableLED(Constants.LedConstants.LED_PORT);
        m_Buffer = new AddressableLEDBuffer(Constants.LedConstants.LED_BUFFER_LENGTH);
        m_led.setLength(m_Buffer.getLength());
        m_led.setData(m_Buffer);
        m_led.start();
    }

    @Override
    public void periodic() {
    }

    public void setled(int R, int G, int B) {
        for (var i = 0; i < m_Buffer.getLength(); i++) {
            m_Buffer.setRGB(i, R, G, B);
        }
        m_led.setData(m_Buffer);
    }
}
