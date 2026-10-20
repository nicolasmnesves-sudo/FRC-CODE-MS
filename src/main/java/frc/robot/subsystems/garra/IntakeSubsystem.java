package frc.robot.subsystems.garra;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class IntakeSubsystem extends SubsystemBase {

 
  private final SparkMax intakeMotor = new SparkMax(52, MotorType.kBrushless);
  private final SparkMax intakeFollowerMotor = new SparkMax(51, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.3, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.3, 0))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(3))
      .withMotorInverted(false)
      .withOpenLoopRampRate(Seconds.of(0.1))
      .withFollowers(new Pair<Object, Boolean>(intakeFollowerMotor, false));

  private final SmartMotorController motor = new SparkWrapper(intakeMotor, DCMotor.getNEO(1), motorConfig);
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

  public Command setRPM(double rpm) {
    return intake.run(RPM.of(rpm));
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

