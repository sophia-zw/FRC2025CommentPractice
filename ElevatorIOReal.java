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
    //defining motors and seuch
    private final TalonFX elevMotor1;
    private final TalonFX elevMotor2;
    //i have no clue what a follower or a motionmagicvoltage is, maybe its the gyroscope?
    private final Follower elevFollower;
    private MotionMagicVoltage position = new MotionMagicVoltage(0);
    //defining signals for voltage, position, velocity, and the current
    private StatusSignal<Voltage> elevVol;
    private StatusSignal<Angle> elevPos;
    private StatusSignal<AngularVelocity> elevVel;
    private StatusSignal<Current> elevCur;
    //constructor
    public ElevatorIOReal(){
        //initializing the motors as talonFX motors, and telling them what ports they are in
        elevMotor1 = new TalonFX(Ports.ELEVATOR_BOTTOM);
        elevMotor2 = new TalonFX(Ports.ELEVATOR_TOP);
        elevFollower = new Follower(elevMotor1.getDeviceID(), false);
        //initializing the status signals 
        elevVol = elevMotor1.getMotorVoltage();
        elevPos = elevMotor1.getPosition(); // Is actually inches
        elevVel = elevMotor1.getVelocity(); // Is actually inches/sec
        elevCur = elevMotor1.getStatorCurrent();

        elevMotor1.getConfigurator().apply(ElevatorConstants.elev1Config);
        elevMotor2.getConfigurator().apply(ElevatorConstants.elev2Config);

        BaseStatusSignal.setUpdateFrequencyForAll(50, elevVol, elevPos, elevVel, elevCur); 
        ParentDevice.optimizeBusUtilizationForAll(elevMotor1,elevMotor2);
        
        elevMotor2.setControl(elevFollower);
    }
    //this updates the inputs on this side to keep elevator.java and this in sync
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        var elev1Status = BaseStatusSignal.refreshAll(elevVol, elevPos, elevVel, elevCur);

        inputs.elev1Connected = elev1Status.isOK();
        inputs.elev2Connected = elevMotor2.isConnected();
        inputs.elevVelocityInchesPerSec = elevVel.getValueAsDouble();
        inputs.elevPositionInches = elevPos.getValueAsDouble();
        inputs.elevVoltage = elevVol.getValue().in(Volts);
        inputs.elevCurrent = elevCur.getValue().in(Amps);
    }
    //yeah so no idea what this does, i think this puts the motor into a closed loop to spin until the elevator gets to the specific position?
    @Override
    public void setElevatorClosedLoop(double pos){ 
        elevMotor1.setControl(position.withPosition(pos)); 
    }
    //this just gives the motor the voltage and has it spin
    @Override
    public void setElevatorOpenLoop(double volts){
        elevMotor1.setVoltage(volts);
    }
    //this tells the motor to spin until the specific start value so it resets the motor
    @Override
    public void resetState() {
        elevMotor1.setPosition(ElevatorPosition.START.value); 
    }
    //this stops all the motors
    @Override
    public void stopElevator(){
        elevMotor1.stopMotor();
    }
}
