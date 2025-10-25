package frc.robot.subsystems.elevator; //N/A


import static edu.wpi.first.units.Units.Inches; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Inches
import static edu.wpi.first.units.Units.Meters; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Meters

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;  //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html
import com.ctre.phoenix6.configs.FeedbackConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/FeedbackConfigs.html
import com.ctre.phoenix6.configs.MotionMagicConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/MotionMagicConfigs.html
import com.ctre.phoenix6.configs.MotorOutputConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html
import com.ctre.phoenix6.configs.Slot0Configs;//https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/Slot0Configs.html
import com.ctre.phoenix6.configs.TalonFXConfiguration; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/TalonFXConfiguration.html
import com.ctre.phoenix6.signals.GravityTypeValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/GravityTypeValue.html
import com.ctre.phoenix6.signals.InvertedValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/InvertedValue.html
import com.ctre.phoenix6.signals.NeutralModeValue;//https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/NeutralModeValue.html
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/StaticFeedforwardSignValue.html


public class ElevatorConstants {
public enum ElevatorPosition { // Just a bunch of constants related to elevator positioning, timing sensing, etc.
        START(0.0), 
        HOLD(0.0),
        ROTATE(19.5),
        LOAD(15.77724609375),
        PICKUP(0.0),
        CORAL_L1(Inches.convertFrom(0.56608017578125, Meters)+1.0),
        CORAL_L2(1.0),
        CORAL_L3(15.936),
        CORAL_L4(Inches.convertFrom(0.9650139648437499, Meters)+2.5),
        CORAL_L4_2(Inches.convertFrom(0.9650139648437499, Meters)+6.0),
        ALGAE_L2(21.614),
        ALGAE_L3(32.669),
        ALGAE_PROCESSOR(0.0),
        ALGAE_NET(0.0);

        public final double value;
        private ElevatorPosition(double value){
            this.value = value;
         }
    }

    public static final double elevatorTolerance = 0.2;

    public static final TalonFXConfiguration elev1Config = new TalonFXConfiguration() //initalizing talonFX motr
    .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80))
        .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(15.0/22/0.25/2)) // 15:1 gear ratio, 22 tetth per rev at 0.25" pitch
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicJerk(2000)
            .withMotionMagicAcceleration(300.0)
            .withMotionMagicCruiseVelocity(200.0))
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast))
        .withSlot0(new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withKS(0.088463)
            .withKV(0.1499517399)
            .withKA(0.01200838201)
            .withKG(0.18429)
            .withKP(3.89255778512)
            .withKI(0.0)
            .withKD(0.2));

    public static final TalonFXConfiguration elev2Config = new TalonFXConfiguration()
    .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLowerLimit(40))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast));
}


    


