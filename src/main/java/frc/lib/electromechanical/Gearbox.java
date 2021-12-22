package frc.lib.electromechanical;

public class Gearbox {
    private double m_outputRatio;

    /**
     * Default output efficiency is set to 0.8
     * This is strictly used for models
     */
    private double m_efficiency = 0.8;

    /**
     * Generic helper class for gearboxes on mechanisms
     * 
     * @param gearing 
     */
    public Gearbox(double inputRatio, double outputRatio) {
        m_outputRatio = inputRatio / outputRatio;
    }

    public Gearbox(double ratio) {
        m_outputRatio = ratio;
	}

	public Gearbox withEfficiency(double efficiency) {
        m_efficiency = efficiency;
        return this;
    }

    public double efficiency() {
        return m_efficiency;
    }

    public double outputRatio() {
        return m_outputRatio;
    }
}
