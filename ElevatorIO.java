package frc.robot.subsystems.elevator; //N/A

import org.littletonrobotics.junction.AutoLog; //N/A

public interface ElevatorIO{ //This entire class is just an interface for ElevatorIOReal to implement. Not much to explain, any class that implemenets this has to use the methods defined (kind of like extending an abstract class withut defining the abstract methods).
    @AutoLog
    public static class ElevatorIOInputs{
        public boolean elev1Connected = false;
        public boolean elev2Connected = false;

        public double elevPositionInches = 0;
        public double elevVelocityInchesPerSec = 0;
        public double elevVoltage = 0;
        public double elevCurrent = 0;

    }

    public default void updateInputs(ElevatorIOInputs inputs){}

    public default void setElevatorClosedLoop(double pos){}
    
    public default void setElevatorOpenLoop(double volts){}

    public default void stopElevator(){} 

	public default void resetState(){} 
}
   

