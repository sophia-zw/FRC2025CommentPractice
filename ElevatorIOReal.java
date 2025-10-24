package frc.robot.subsystems.elevator;  //N/A

import static edu.wpi.first.units.Units.Amps; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Amps
import static edu.wpi.first.units.Units.Volts; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Volts

import com.ctre.phoenix6.BaseStatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/BaseStatusSignal.html
import com.ctre.phoenix6.StatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/StatusSignal.html
import com.ctre.phoenix6.controls.Follower; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/Follower.html
import com.ctre.phoenix6.controls.MotionMagicVoltage; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/MotionMagicVoltage.html
import com.ctre.phoenix6.hardware.ParentDevice; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/ParentDevice.html
import com.ctre.phoenix6.hardware.TalonFX;//https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/TalonFX.html

import edu.wpi.first.units.measure.Angle;//https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Angle.html
import edu.wpi.first.units.measure.AngularVelocity; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/AngularVelocity.html
import edu.wpi.first.units.measure.Current; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Current.html
import edu.wpi.first.units.measure.Voltage; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Voltage.html
import frc.robot.Constants.Ports; //N/A
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition; //N/A

public class ElevatorIOReal implements ElevatorIO{
    private final TalonFX elevMotor1;
    private final TalonFX elevMotor2;
    private final Follower elevFollower;
    private MotionMagicVoltage position = new MotionMagicVoltage(0);

    private StatusSignal<Voltage> elevVol;
    private StatusSignal<Angle> elevPos;
    private StatusSignal<AngularVelocity> elevVel;
    private StatusSignal<Current> elevCur;

    public ElevatorIOReal(){
        elevMotor1 = new TalonFX(Ports.ELEVATOR_BOTTOM); // These 2 lines initialize motor variables
        elevMotor2 = new TalonFX(Ports.ELEVATOR_TOP);
        elevFollower = new Follower(elevMotor1.getDeviceID(), false); 

        elevVol = elevMotor1.getMotorVoltage(); // All variables just storing details about motors
        elevPos = elevMotor1.getPosition(); // Is actually inches
        elevVel = elevMotor1.getVelocity(); // Is actually inches/sec
        elevCur = elevMotor1.getStatorCurrent();

        elevMotor1.getConfigurator().apply(ElevatorConstants.elev1Config); //Applies Configurator settings from constants relating to PID, limits, etc.
        elevMotor2.getConfigurator().apply(ElevatorConstants.elev2Config);

        BaseStatusSignal.setUpdateFrequencyForAll(50, elevVol, elevPos, elevVel, elevCur); //sets the speed that all these variables that store the current status of the motors will be renewed
        ParentDevice.optimizeBusUtilizationForAll(elevMotor1,elevMotor2); //optimizes the motors
        
        elevMotor2.setControl(elevFollower); // makes the second motor follow the first?
    }
    
    @Override

    //Stores all motor data, checks connection status of motors, refreshes all current motor details and renews it with the current details.
    public void updateInputs(ElevatorIOInputs inputs){
        var elev1Status = BaseStatusSignal.refreshAll(elevVol, elevPos, elevVel, elevCur);

        inputs.elev1Connected = elev1Status.isOK();
        inputs.elev2Connected = elevMotor2.isConnected();
        inputs.elevVelocityInchesPerSec = elevVel.getValueAsDouble();
        inputs.elevPositionInches = elevPos.getValueAsDouble();
        inputs.elevVoltage = elevVol.getValue().in(Volts);
        inputs.elevCurrent = elevCur.getValue().in(Amps);
    }

    @Override
    //Gives voltage to the motor based on feedback and other factors because its a closed loop
    public void setElevatorClosedLoop(double pos){ //
        elevMotor1.setControl(position.withPosition(pos)); 
    }

    @Override
    //Gives voltage to the motor moving the elevator effectively creating an open loop(moves without receiving feedback and doesn't adjust based on external circumstances)
    public void setElevatorOpenLoop(double volts){
        elevMotor1.setVoltage(volts);
    }

    @Override
    //Sets the position of the elevator to the initial Start Position
    public void resetState() {
        elevMotor1.setPosition(ElevatorPosition.START.value); 
    }

    @Override
    //Stops the voltage to the motor controlling the elevator
    public void stopElevator(){
        elevMotor1.stopMotor();
    }
}

