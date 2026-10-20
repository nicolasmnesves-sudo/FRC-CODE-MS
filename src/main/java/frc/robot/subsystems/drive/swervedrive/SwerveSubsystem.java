// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swervedrive;

// ============ IMPORTS: STATIC / PATHPLANNER =================
// Unidades do WPILib e classes do PathPlanner (autônomo e pathfinding).
// PathPlannerPath só é usado nos comentários de documentação.
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

// ============ IMPORTS: WPILIB =================
// Matemática (poses, cinemática), driver station, timer e Command-Based.
// VecBuilder só é necessário se a fusão com a Limelight for reativada.
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
// import edu.wpi.first.math.VecBuilder;               // usado só no periodic() comentado (visão)

// ============ IMPORTS: PROJETO E JAVA =================
// Classes do próprio robô e utilitários padrão do Java.
// LimelightPoseEstimator só é usado no periodic() comentado.
import frc.robot.constants.*;
import frc.robot.commands.swervedrive.drivebase.DriveRelativeTranslationCommand;
// import frc.robot.limelight.LimelightPoseEstimator;  // usado só no periodic() comentado (visão)

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

// ============ IMPORTS: YAGSL (SWERVELIB) =================
// Biblioteca que controla os módulos swerve, a odometria e a telemetria.
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// ============ CLASSE SwerveSubsystem =================
// Subsistema de tração: envolve o SwerveDrive do YAGSL e expõe comandos e métodos.
// Todo o resto do robô usa esta classe para mover o chassi e ler a pose.
public class SwerveSubsystem extends SubsystemBase
{

    // ####################################################################
    // SEÇÃO 1 - ATRIBUTOS
    // ####################################################################

    // ============ OBJETO SWERVE (YAGSL) =================
    // Controla os 4 módulos, o giroscópio e a odometria.
    // É criado uma única vez no construtor e nunca trocado (final).
    private final SwerveDrive swerveDrive;

    // ============ ESTIMADOR DE POSE DA LIMELIGHT (DESATIVADO) =================
    // Era usado para corrigir a odometria com visão (MegaTag2).
    // Está comentado porque o projeto não usará AprilTags.
    // private LimelightPoseEstimator limelightPoseEstimator = new LimelightPoseEstimator();

    // ####################################################################
    // SEÇÃO 2 - CONSTRUTORES
    // ####################################################################

    // ============ CONSTRUTOR PRINCIPAL (LÊ OS JSON DO SWERVE) =================
    // Cria o SwerveDrive a partir da pasta deploy/swerve e configura as compensações.
    // É o construtor usado pelo RobotContainer.
    public SwerveSubsystem(File directory)
    {
        // Pose inicial fixa por aliança. ATENÇÃO: blueAlliance está fixo em false,
        // então sempre começa em (16 m, 4 m) a 180°, ou (14 m, 6 m).
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                          Meter.of(4)),
                                                        Rotation2d.fromDegrees(0))
                                           : new Pose2d(new Translation2d(Meter.of(14),
                                                                          Meter.of(6)),
                                                        Rotation2d.fromDegrees(180));

        // Telemetria precisa ser configurada ANTES de criar o SwerveDrive,
        // para evitar a criação de objetos desnecessários.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Cria o swerve lendo os JSON (módulos, física, PID) da pasta indicada.
        try
        {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
            // Alternativa, se os fatores de conversão não vierem dos JSON:
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        // Correção de heading: desligada (só faz sentido ao controlar o robô por ângulo).
        swerveDrive.setHeadingCorrection(false);

        // Compensação de cosseno: desligada (útil só para evitar diferenças na simulação).
        swerveDrive.setCosineCompensator(false);

        // Compensa o desvio lateral que cresce com a velocidade angular (coeficiente inicial 0.1).
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

        // Ressincronização automática dos encoders absolutos: desligada.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        // swerveDrive.pushOffsetsToEncoders(); // usaria o encoder absoluto com os offsets gravados nele

        // Configura o AutoBuilder do PathPlanner (necessário para autônomo e pathfinding).
        setupPathPlanner();
        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

    // ============ CONSTRUTOR ALTERNATIVO (NÃO USADO) =================
    // Cria o swerve direto de objetos de configuração, sem ler JSON.
    // Pose inicial fixa em (2 m, 0 m); não chama o setupPathPlanner.
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
    {
        swerveDrive = new SwerveDrive(driveCfg,
                                      controllerCfg,
                                      Constants.MAX_SPEED,
                                      new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                                 Rotation2d.fromDegrees(0)));
    }

    // ####################################################################
    // SEÇÃO 3 - CICLO PERIÓDICO
    // ####################################################################

    // ============ periodic() (DESATIVADO) =================
    // Atualizaria a odometria e fundiria a pose da Limelight (MegaTag2) a cada ciclo.
    // Antes de usar, confirme se a versão do YAGSL já atualiza a odometria sozinha.
    // @Override
    // public void periodic() {
    //     swerveDrive.updateOdometry();
    //     limelightPoseEstimator.updateEstimatePose(
    //         swerveDrive.getOdometryHeading().getDegrees(),
    //         swerveDrive.getRobotVelocity().omegaRadiansPerSecond * (180 / Math.PI));
    //     if (limelightPoseEstimator.isTheLastEstimatedPoseValid()) {
    //         Pose2d mt2Pose2d = limelightPoseEstimator.getEstimatedPose();
    //         swerveDrive.addVisionMeasurement(mt2Pose2d,
    //             limelightPoseEstimator.getTimestampSecondsEstimatedPose(),
    //             VecBuilder.fill(.7, .7, 9999999));
    //     }
    // }

    // ============ simulationPeriodic() =================
    // Chamado a cada ciclo apenas em simulação.
    // Está vazio: o YAGSL já cuida da simulação internamente.
    @Override
    public void simulationPeriodic()
    {
    }

    // ####################################################################
    // SEÇÃO 4 - PATHPLANNER E AUTÔNOMO
    // ####################################################################

    // ============ CONFIGURAR O AUTOBUILDER DO PATHPLANNER =================
    // Informa ao PathPlanner como ler a pose, resetar a odometria e dirigir o robô.
    // Usa o PID holonômico (5, 0, 0) e lê as medidas do robô de settings.json.
    public void setupPathPlanner()
    {
        RobotConfig config;
        try
        {
            // Lê massa, tamanho, motores e limites do arquivo deploy/pathplanner/settings.json.
            config = RobotConfig.fromGUISettings();

            // Quando true, repassa as forças de feedforward de cada módulo ao drive.
            final boolean enableFeedforward = true;

            AutoBuilder.configure(
                // Pose atual do robô.
                this::getPose,
                // Reseta a odometria (chamado quando o auto tem uma pose inicial).
                this::resetOdometry,
                // Velocidade atual do chassi. DEVE ser relativa ao robô.
                this::getRobotVelocity,
                // Dirige o robô com velocidades relativas ao robô (com ou sem feedforward).
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (enableFeedforward)
                    {
                        swerveDrive.drive(
                            speedsRobotRelative,
                            swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces());
                    } else
                    {
                        swerveDrive.setChassisSpeeds(speedsRobotRelative);
                    }
                },
                // Controlador de seguimento de trajetória: PID de translação e de rotação.
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),   // translação
                    new PIDConstants(5.0, 0.0, 0.0)),  // rotação
                config,
                // Espelha o path quando a aliança é vermelha (a origem continua no lado azul).
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent())
                    {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                // Subsistema que o PathPlanner vai "requisitar".
                this);

        } catch (Exception e)
        {
            // Não engole o erro: se falhar, o AutoBuilder fica sem configurar e
            // buildAutoChooser/pathfindToPose quebram longe daqui.
            DriverStation.reportError("setupPathPlanner falhou: " + e, e.getStackTrace());
            e.printStackTrace();
        }

        // Pré-carrega o pathfinding para o primeiro uso não travar o robô.
        PathfindingCommand.warmupCommand().schedule();
    }

    // ============ COMANDO DE AUTÔNOMO PELO NOME =================
    // Cria um PathPlannerAuto a partir do nome do auto salvo no PathPlanner.
    // Também dispara os event markers do path.
    public Command getAutonomousCommand(String pathName)
    {
        return new PathPlannerAuto(pathName);
    }

    // ============ IR ATÉ UMA POSE NO CAMPO (PATHFINDING) =================
    // Usa o pathfinding do PathPlanner para chegar a uma pose, desviando de obstáculos.
    // Depende do navgrid.json e do AutoBuilder já configurado.
    public Command driveToPose(Pose2d pose)
    {
        // Limites do deslocamento: velocidade máx., 4 m/s², velocidade angular máx., 720°/s².
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(), 4.0,
            swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Termina parado (velocidade final 0 m/s).
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0));
    }

    // ============ COMANDO: ANDAR UMA DISTÂNCIA RELATIVA AO ROBÔ =================
    // deltaX = frente (+) / trás (-); deltaY = esquerda (+) / direita (-), em metros.
    // Usa o DriveRelativeTranslationCommand (PID próprio sobre a odometria).
    public Command createDriveRelativeCommand(double deltaX, double deltaY)
    {
        return new DriveRelativeTranslationCommand(this, deltaX, deltaY);
    }

    // ####################################################################
    // SEÇÃO 5 - COMANDOS DE DIREÇÃO (JOYSTICK)
    // ####################################################################

    // ============ COMANDO: DIRIGIR COM VELOCIDADE ANGULAR (JOYSTICK) =================
    // Os dois primeiros eixos controlam a translação; o terceiro, a rotação (ao cubo).
    // Versão do template; hoje o RobotContainer usa o SwerveInputStream.
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                  translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                  translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                              Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                              true,    // campo-orientado
                              false);  // malha fechada
        });
    }

    // ============ COMANDO: DIRIGIR COM ÂNGULO-ALVO (JOYSTICK) =================
    // O stick direito (headingX/headingY) define o ângulo que o robô deve apontar.
    // Idealmente usado com setHeadingCorrection(true). Versão do template.
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                                DoubleSupplier headingY)
    {
        return run(() -> {
            // Reduz a translação a 80 % para suavizar o controle.
            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                       translationY.getAsDouble()), 0.8);

            // Calcula as velocidades-alvo e dirige em modo campo-orientado.
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                            headingX.getAsDouble(),
                                                                            headingY.getAsDouble(),
                                                                            swerveDrive.getOdometryHeading().getRadians(),
                                                                            swerveDrive.getMaximumChassisVelocity()));
        });
    }

    // ============ COMANDO: DIRIGIR CAMPO-ORIENTADO COM SUPPLIER =================
    // Executa a cada ciclo as velocidades field-relative fornecidas pelo supplier.
    // É o comando padrão do drivetrain (alimentado pelo SwerveInputStream).
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
    {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    // ============ COMANDO: DIRIGIR COM SETPOINT GENERATOR (254) =================
    // Versão field-relative do gerador de setpoints, que respeita limites de aceleração.
    // Retorna Commands.none() se as configurações do PathPlanner falharem.
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
    {
        try
        {
            // Converte de campo-orientado para robô-orientado antes de gerar o setpoint.
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
            });
        } catch (Exception e)
        {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();
    }

    // ============ SETPOINT GENERATOR (IMPLEMENTAÇÃO INTERNA) =================
    // Gera estados de módulo suaves a partir de velocidades robô-orientadas (porte da equipe 254).
    // Lança IOException/ParseException se o settings.json do PathPlanner for inválido.
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
    throws IOException, ParseException
    {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                                swerveDrive.getMaximumChassisAngularVelocity());

        // Guarda o último setpoint e o instante anterior (começa com o estado atual do robô).
        AtomicReference<SwerveSetpoint> prevSetpoint
            = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                       swerveDrive.getStates(),
                                                       DriveFeedforwards.zeros(swerveDrive.getModules().length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        // Ao iniciar, marca o tempo; a cada ciclo gera e aplica o novo setpoint.
        return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                        () -> {
                            double newTime = Timer.getFPGATimestamp();
                            SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                            robotRelativeChassisSpeed.get(),
                                                                                            newTime - previousTime.get());
                            swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                              newSetpoint.moduleStates(),
                                              newSetpoint.feedforwards().linearForces());
                            prevSetpoint.set(newSetpoint);
                            previousTime.set(newTime);
                        });
    }

    // ============ COMANDO: CENTRALIZAR OS MÓDULOS =================
    // Aponta as 4 rodas para 0° (frente) enquanto o comando estiver rodando.
    // Útil para conferir os offsets dos encoders.
    public Command centerModulesCommand()
    {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                               .forEach(it -> it.setAngle(0.0)));
    }

    // ####################################################################
    // SEÇÃO 6 - CONTROLE DIRETO DO CHASSI (SEM COMANDO)
    // ####################################################################

    // ============ DRIVE PRINCIPAL (TRANSLAÇÃO + ROTAÇÃO) =================
    // translation em m/s, rotation em rad/s (anti-horário positivo); fieldRelative escolhe o referencial.
    // Robô-orientado: x = frente, y = esquerda. Campo-orientado: x = longe da aliança.
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                          rotation,
                          fieldRelative,
                          false); // malha aberta desligada (quase nunca deve ser usada)
    }

    // ============ DRIVE CAMPO-ORIENTADO (CHASSISSPEEDS) =================
    // Dirige o robô com uma velocidade expressa no referencial do campo.
    // Executa uma única vez; use dentro de um comando para repetir.
    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }

    // ============ DRIVE ROBÔ-ORIENTADO (CHASSISSPEEDS) =================
    // Dirige o robô com uma velocidade expressa no referencial do próprio robô.
    // Executa uma única vez; use dentro de um comando para repetir.
    public void drive(ChassisSpeeds velocity)
    {
        swerveDrive.drive(velocity);
    }

    // ============ DEFINIR VELOCIDADES DO CHASSI (MALHA FECHADA) =================
    // Aplica as velocidades com controle de velocidade em malha fechada nos módulos.
    // Usado, por exemplo, pelo PathPlanner sem feedforward.
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    // ####################################################################
    // SEÇÃO 7 - CONVERSÃO DE JOYSTICK PARA VELOCIDADES
    // ####################################################################

    // ============ VELOCIDADES-ALVO: STICK ESQUERDO + STICK DIREITO (ÂNGULO) =================
    // xInput/yInput = translação; headingX/headingY = direção em que o robô deve apontar.
    // Aplica o cubo na translação para suavizar o controle.
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                            scaledInputs.getY(),
                                                            headingX,
                                                            headingY,
                                                            getHeading().getRadians(),
                                                            Constants.MAX_SPEED);
    }

    // ============ VELOCIDADES-ALVO: STICK + ÂNGULO FIXO =================
    // xInput/yInput = translação; angle = ângulo em que o robô deve ficar (Rotation2d).
    // Aplica o cubo na translação para suavizar o controle.
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
    {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                            scaledInputs.getY(),
                                                            angle.getRadians(),
                                                            getHeading().getRadians(),
                                                            Constants.MAX_SPEED);
    }

    // ####################################################################
    // SEÇÃO 8 - ESTADO DO ROBÔ (POSE E VELOCIDADE)
    // ####################################################################

    // ============ LER A POSE ATUAL =================
    // Retorna posição (x, y) e ângulo do robô, segundo a odometria.
    // É a base para qualquer navegação.
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    // ============ RESETAR A ODOMETRIA =================
    // Define a pose atual como a pose informada.
    // Não precisa zerar o giroscópio nem os módulos antes; se zerar, chame este método de novo.
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    // ============ LER O ÂNGULO (HEADING) DO ROBÔ =================
    // Vem do estimador de pose, não da leitura crua do giroscópio.
    // Pode ter sido corrigido por chamadas a resetOdometry().
    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    // ============ LER O PITCH (INCLINAÇÃO) =================
    // Ângulo de inclinação do robô, medido pela IMU.
    // Útil para detectar rampa ou tombamento.
    public Rotation2d getPitch()
    {
        return swerveDrive.getPitch();
    }

    // ============ VELOCIDADE ROBÔ-ORIENTADA =================
    // Velocidade atual (x, y e omega) no referencial do robô.
    // É o que o PathPlanner espera receber.
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    // ============ VELOCIDADE CAMPO-ORIENTADA =================
    // Velocidade atual (x, y e omega) no referencial do campo.
    // Útil para telemetria e para comandos de navegação.
    public ChassisSpeeds getFieldVelocity()
    {
        return swerveDrive.getFieldVelocity();
    }

    // ============ CINEMÁTICA DO SWERVE =================
    // Retorna o objeto de cinemática (posições dos módulos).
    // Converte velocidades do chassi em estados de módulo e vice-versa.
    public SwerveDriveKinematics getKinematics()
    {
        return swerveDrive.kinematics;
    }

    // ============ PUBLICAR UMA TRAJETÓRIA NO CAMPO =================
    // Envia a trajetória para o Field2d (visualização no dashboard).
    // Só afeta a telemetria, não move o robô.
    public void postTrajectory(Trajectory trajectory)
    {
        swerveDrive.postTrajectory(trajectory);
    }

    // ####################################################################
    // SEÇÃO 9 - GIROSCÓPIO E ALIANÇA
    // ####################################################################

    // ============ ZERAR O GIROSCÓPIO =================
    // Define o ângulo atual como 0° e reposiciona a odometria no mesmo ponto virado para 0°.
    // Hoje nenhum botão chama este método.
    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
    }

    // ============ VERIFICAR SE A ALIANÇA É VERMELHA =================
    // Lê a aliança do Driver Station; retorna false se ela ainda não estiver disponível.
    // É privado: só o próprio subsistema usa.
    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    // ============ ZERAR O GIROSCÓPIO CONSIDERANDO A ALIANÇA =================
    // Assume que o robô está virado para frente; se a aliança é vermelha, gira a pose em 180°.
    // Mantém a posição (x, y) atual.
    public void zeroGyroWithAlliance()
    {
        if (isRedAlliance())
        {
            zeroGyro();
            // Depois de zerar, ajusta a pose para 180°.
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else
        {
            zeroGyro();
        }
    }

    // ####################################################################
    // SEÇÃO 10 - FREIO E TRAVA
    // ####################################################################

    // ============ FREIO / COAST DOS MOTORES =================
    // true = freio (o robô resiste a ser empurrado); false = coast (gira livre).
    // Usado pelo Robot.java ao habilitar e desabilitar.
    public void setMotorBrake(boolean brake)
    {
        swerveDrive.setMotorIdleMode(brake);
    }

    // ============ TRAVAR AS RODAS EM "X" =================
    // Posiciona os módulos em X para o robô não se mover.
    // Deve ser chamado de forma contínua para manter a trava.
    public void lock()
    {
        swerveDrive.lockPose();
    }

    // ####################################################################
    // SEÇÃO 11 - CALIBRAÇÃO E DIAGNÓSTICO
    // ####################################################################

    // ============ SYSID: CARACTERIZAR OS MOTORES DE DRIVE =================
    // Roda a rotina do SysId nos motores de tração (quase-estático e dinâmico).
    // Os dados servem para calcular kS, kV e kA.
    public Command sysIdDriveMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                new Config(),
                this, swerveDrive, 12, true),
            3.0, 5.0, 3.0);
    }

    // ============ SYSID: CARACTERIZAR OS MOTORES DE ÂNGULO =================
    // Roda a rotina do SysId nos motores que giram as rodas.
    // Os dados servem para ajustar o controle de ângulo.
    public Command sysIdAngleMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                new Config(),
                this, swerveDrive),
            3.0, 5.0, 3.0);
    }

    // ============ TROCAR O FEEDFORWARD DOS MÓDULOS =================
    // Substitui o feedforward do drive por um novo SimpleMotorFeedforward (kS, kV, kA).
    // Use com os valores obtidos no SysId.
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
    {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    // ============ MEDIDA DE VISÃO FALSA (TESTE) =================
    // Injeta uma pose falsa (3, 3) a 65° no estimador, só para testar a fusão de visão.
    // Não deve ser usado em partida.
    public void addFakeVisionReading()
    {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    // ####################################################################
    // SEÇÃO 12 - ACESSO AOS OBJETOS INTERNOS DO YAGSL
    // ####################################################################

    // ============ OBTER O SWERVECONTROLLER =================
    // Retorna o controlador que converte entrada de joystick em velocidades.
    // Usado por comandos como o AbsoluteDrive.
    public SwerveController getSwerveController()
    {
        return swerveDrive.swerveController;
    }

    // ============ OBTER A CONFIGURAÇÃO DO SWERVE =================
    // Retorna a configuração lida dos JSON (módulos, IMU, física).
    // Útil para consultar dimensões e parâmetros.
    public SwerveDriveConfiguration getSwerveDriveConfiguration()
    {
        return swerveDrive.swerveDriveConfiguration;
    }

    // ============ OBTER O SWERVEDRIVE =================
    // Retorna o objeto principal do YAGSL, para quem precisa de acesso direto.
    // Prefira os métodos desta classe sempre que possível.
    public SwerveDrive getSwerveDrive()
    {
        return swerveDrive;
    }
}