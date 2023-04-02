package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    private final CANifier canifier;
    private double colorR = 255;
    private double colorG = 0;
    private double colorB = 0;

    public LEDSubsystem() {
        canifier = new CANifier(11);
    }

    private void setBluePourcentage(double pourcentage) {
        canifier.setLEDOutput(pourcentage / 2, LEDChannel.LEDChannelA);
    }

    private void setRedPourcentage(double pourcentage) {
        canifier.setLEDOutput(pourcentage / 2, LEDChannel.LEDChannelB);
    }

    private void setGreenPourcentage(double pourcentage) {
        canifier.setLEDOutput(pourcentage / 2, LEDChannel.LEDChannelC);
    }

    public void setRGB(double R, double G, double B) {
        setRedPourcentage(R / 255);
        setGreenPourcentage(G / 255);
        setBluePourcentage(B / 255);
        colorR = R;
        colorG = G;
        colorB = B;
    }

    public void turnOff() {
        setRedPourcentage(0);
        setGreenPourcentage(0);
        setBluePourcentage(0);
    }

    public boolean isEnabled() {
        return getR() != 0 || getG() != 0 || getB() != 0;
    }

    public double getR() {
        return colorR;
    }

    public double getG() {
        return colorG;
    }

    public double getB() {
        return colorB;
    }
}