package frc.robot.subsystems.elevator; //N/A
import org.littletonrobotics.junction.AutoLogOutput; //N/A
import org.littletonrobotics.junction.Logger; //N/A

import edu.wpi.first.wpilibj2.command.Command; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html
import edu.wpi.first.wpilibj2.command.SubsystemBase; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutine.html
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition; //N/A
import edu.wpi.first.wpilibj.Alert; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Alert.html
import edu.wpi.first.wpilibj.Alert.AlertType; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Alert.AlertType.html

import static edu.wpi.first.units.Units.*; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html


//class declaration and stuff, it is a subclass of subsytem base
public class Elevator extends SubsystemBase {
    //constants and stuff, elevatorio for the input output
    private final ElevatorIO elevatorIO;
    //couldnt find anything about this class in other files, however i think it has something to do with the littletonrobotics library with autologging
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    //looks like this is defining some alerts
    private final Alert elev1DisconnectedAlert, elev2DisconnectedAlert;
    //i think this annotation just means it logs it automatically, also defining the position of where it wants to go
    @AutoLogOutput
    private ElevatorPosition targetPosition;

    private final SysIdRoutine sysId;
    //constructor, needs and elevator IO
    public Elevator(ElevatorIO io){
        elevatorIO = io;
        //initializing the alerts, for each motor disconnecting
        elev1DisconnectedAlert = new Alert("Disconnected elevator motor 1.", AlertType.kError); 
        elev2DisconnectedAlert = new Alert("Disconnected elevator motor 2.", AlertType.kError); 
        //by default be at the start position
        targetPosition = ElevatorPosition.START;
        //this is just for tests and stuff i think
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(0.25),
                    Volts.of(2),
                    Seconds.of(6),
                    state -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> runCharacterization(volts.in(Volts)), null, this));
    }
    //basically this function runs constantly, similar to a tick in a game, this is where we are updating our inputs and something to do with our alerts
    @Override
    public void periodic(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        elev1DisconnectedAlert.set(!inputs.elev1Connected);
        elev2DisconnectedAlert.set(!inputs.elev2Connected);
    }
    //this sets the position of the elevator
    public void setPosition(ElevatorPosition height){
        elevatorIO.setElevatorClosedLoop(height.value);
        targetPosition = height; 
    }
    //checks if the elevator is at the target position
    @AutoLogOutput
    public boolean isAtPosition() {
        return isAtPosition(targetPosition);
    }
    //checks if the elevator is at the position that is put into the function, also it does something with elevator tolerance
    public boolean isAtPosition(ElevatorPosition position) {
        return Math.abs(inputs.elevPositionInches - position.value) < ElevatorConstants.elevatorTolerance;
    }
    //returns where the elevator is trying to go
    public ElevatorPosition getTargetPosition(){
        return targetPosition;
    }
    //returns the current elevator height in inches
    public double getHeight(){
        return inputs.elevPositionInches;
    }
    //checks to see if the elevator is currently not doing any commands, like when you ask a friend if they are free to hangout
    @AutoLogOutput
    public boolean isFree() {
        return this.getCurrentCommand() == null;
    }
    //gets the current command it is doing, if its null then it will say NONE
    @AutoLogOutput
    public String current() {
        return this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE";
    }
    //this stops the elevator, quite self explanatory
    public void stopElevator(){  
        elevatorIO.stopElevator();
    }
    //Uhhhhhh, i think this is do with like testing something
    public void runCharacterization(double volts) {
        elevatorIO.setElevatorOpenLoop(volts);
    }

    /** Returns a command to run a dynamic test in the specified direction. (hey look this was done for me, Neat!)*/
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }
    //this resets the state and sets the position back to the bottom
    public void resetState() {
        elevatorIO.resetState();
        setPosition(ElevatorPosition.START);
    }
}
