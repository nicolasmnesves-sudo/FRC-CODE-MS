package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {

 
  private final TalonFX intakeMotor = new TalonFX(51, CANBus.roboRIO());
  private final TalonFX intakeFollowerMotor = new TalonFX(52, CANBus.roboRIO());

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.43, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.18, 0.31, 0))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(30))
      .withMotorInverted(false)
      .withOpenLoopRampRate(Seconds.of(0.1))
      .withControlMode(ControlMode.OPEN_LOOP)
     //false segue na mesma direção
      .withFollowers(new Pair<Object, Boolean>(intakeFollowerMotor, false));

  private final SmartMotorController motor = new TalonFXWrapper(intakeMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(RPM.of(6000));

  private final FlyWheel intake = new FlyWheel(intakeConfig, motor);

  public IntakeSubsystem() {}

  public AngularVelocity getVelocity() {
    return intake.getSpeed();
  }

  public Command setDutyCycle(double dutyCycle) {
    return intake.set(dutyCycle);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return intake.set(dutyCycle);
  }

  public Command in() {
    return setDutyCycle(0.8);
  }

  public Command out() {
    return setDutyCycle(-0.8);
  }

  public Command stop() {
    return setDutyCycle(0.0);
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }
}
