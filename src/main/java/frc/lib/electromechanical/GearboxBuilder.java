package frc.lib.electromechanical;

public class GearboxBuilder {
    /**
     * Build a gearbox using a simple input output ratio, for example,
     * if the gearbox is known to be '10:1' simply call GeraboxBuilder(10,1)
     * 
     * The output ratio is the amount of times the input spins compared to the output. 
     * For the 10:1 example, this case says that the input spins 10 times for every
     * 1 time that the output spins. So inputRatio = 10, outputRatio = 1
     * 
     * @param inputRatio Gear ratio on the input
     * @param outputRatio Gear ratio on the output
     */
    public GearboxBuilder(double inputRatio, double outputRatio) {

    }

    /**
     * Create basic instance with no gearing
     */
    public GearboxBuilder() {
        
    }

    public class Component {
        private String m_name;

        public Component(String name) {
            m_name = name;
        }

        public void isInput() {

        }

        public void isOutput() {

        }

        public Component coupledTo(String name) {
            return this;
        }

        public Component axleMountedTo(String name) {
            return this;
        }
    }

    public Component addGear(String name, int gearTeeth) {
        return this.new Component(name);
    }

	public Gearbox build() {
        return new Gearbox(1, 1);
	}

}
