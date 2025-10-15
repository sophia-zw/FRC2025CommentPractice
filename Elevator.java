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


public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Alert elev1DisconnectedAlert, elev2DisconnectedAlert;
    @AutoLogOutput
    private ElevatorPosition targetPosition;

    private final SysIdRoutine sysId;

    public Elevator(ElevatorIO io){
        elevatorIO = io;
        elev1DisconnectedAlert = new Alert("Disconnected elevator motor 1.", AlertType.kError); 
        elev2DisconnectedAlert = new Alert("Disconnected elevator motor 2.", AlertType.kError); 
        targetPosition = ElevatorPosition.START;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(0.25),
                    Volts.of(2),
                    Seconds.of(6),
                    state -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> runCharacterization(volts.in(Volts)), null, this));
    }

    @Override
    public void periodic(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        elev1DisconnectedAlert.set(!inputs.elev1Connected);
        elev2DisconnectedAlert.set(!inputs.elev2Connected);
    }

    public void setPosition(ElevatorPosition height){
        elevatorIO.setElevatorClosedLoop(height.value);
        targetPosition = height; 
    }

    @AutoLogOutput
    public boolean isAtPosition() {
        return isAtPosition(targetPosition);
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return Math.abs(inputs.elevPositionInches - position.value) < ElevatorConstants.elevatorTolerance;
    }

    public ElevatorPosition getTargetPosition(){
        return targetPosition;
    }

    public double getHeight(){
        return inputs.elevPositionInches;
    }

    @AutoLogOutput
    public boolean isFree() {
        return this.getCurrentCommand() == null;
    }

    @AutoLogOutput
    public String current() {
        return this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE";
    }

    public void stopElevator(){  
        elevatorIO.stopElevator();
    }

    public void runCharacterization(double volts) {
        elevatorIO.setElevatorOpenLoop(volts);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    public void resetState() {
        elevatorIO.resetState();
        setPosition(ElevatorPosition.START);
    }
}
