package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.DriveRelativeTranslationCommand;
import frc.robot.subsystems.drive.swervedrive.SwerveSubsystem;

import swervelib.SwerveInputStream;


// ============ CONTROLE DO OPERADOR ============
// Inicializa o controle Xbox utilizado para comandar a movimentação do robô.
public class RobotContainer {

    private final CommandXboxController driverXbox =
            new CommandXboxController(0);


    // ============ SUBSISTEMA SWERVE ============
    // Carrega a configuração dos módulos swerve localizada em deploy/swerve/neo.
    // O construtor do SwerveSubsystem também configura o AutoBuilder do PathPlanner.
    private final SwerveSubsystem drivebase =
            new SwerveSubsystem(
                    new File(
                            Filesystem.getDeployDirectory(),
                            "swerve/neo"
                    )
            );


    // ============ SELETOR DE AUTÔNOMO ============
    // Lista no dashboard os autos criados no PathPlanner (pasta deploy/pathplanner/autos).
    // É preenchido em configureAutonomous() e lido em getAutonomousCommand().
    private SendableChooser<Command> autoChooser;


    // ============ TAMANHO DA ARENA (PROVISÓRIO) ============
    // Dimensões do campo do ECOSYNC (14 m x 6 m), usadas para espelhar os paths na aliança vermelha.
    // Mover para um FieldConstants quando ele existir.
    private static final double FIELD_LENGTH_METERS = 14.0;
    private static final double FIELD_WIDTH_METERS  = 6.0;


    // ============ CONDUÇÃO POR VELOCIDADE ANGULAR ============
    // Utiliza o joystick esquerdo para translação e o eixo horizontal direito para rotação.
    private final SwerveInputStream driveAngularVelocity =
            SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
                    () -> -driverXbox.getLeftY(),
                    () -> -driverXbox.getLeftX()
            )
            .withControllerRotationAxis(driverXbox::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(1.0)
            .scaleRotation(1.0)
            .allianceRelativeControl(false);


    // ============ CONDUÇÃO POR ÂNGULO DIRETO ============
    // Utiliza o joystick direito para definir a orientação desejada do robô.
    public final SwerveInputStream driveDirectAngle =
            driveAngularVelocity.copy()
                    .withControllerHeadingAxis(
                            () -> -driverXbox.getRightX(),
                            () -> -driverXbox.getRightY()
                    )
                    .headingWhile(true);


    // ============ CONDUÇÃO EM BAIXA VELOCIDADE ============
    // Reduz a velocidade de translação para facilitar testes e manobras.
    public final SwerveInputStream driveLowSpeed =
            driveDirectAngle.copy()
                    .scaleTranslation(0.4);


    // ============ CONSTRUTOR ============
    // Configura o tamanho do campo, os controles e o autônomo, nesta ordem.
    // O autônomo vem depois do swerve porque o AutoBuilder precisa já estar configurado.
    public RobotContainer() {
        configureFieldSize();
        configureBindings();
        configureAutonomous();

        DriverStation.silenceJoystickConnectionWarning(true);
    }


    // ============ CONFIGURAÇÃO DO TAMANHO DO CAMPO ============
    // Informa ao PathPlanner o tamanho real da arena; o padrão dele é o campo oficial da FIRST.
    // Sem isso, os paths da aliança vermelha são espelhados no lugar errado.
    private void configureFieldSize() {
        FlippingUtil.fieldSizeX = FIELD_LENGTH_METERS;
        FlippingUtil.fieldSizeY = FIELD_WIDTH_METERS;

        // Rotacional = o lado vermelho é o azul girado 180°. É o que a planta parece indicar;
        // confirmar com a organização. Se for espelhado, trocar por FieldSymmetry.kMirrored.
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
    }


    // ============ CONFIGURAÇÃO DOS CONTROLES ============
    // Define o comando padrão de condução e as ações acionadas pelos botões.
    private void configureBindings() {

        // ============ CONDUÇÃO PADRÃO ============
        // Permite controlar a translação e a orientação do robô continuamente.
        drivebase.setDefaultCommand(
                drivebase.driveFieldOriented(driveDirectAngle)
        );


        // ============ DESLOCAMENTO RELATIVO PELO BOTÃO A ============
        // Solicita um deslocamento de 1 metro no eixo X relativo à pose inicial do comando.
        driverXbox.a().onTrue(
                new DriveRelativeTranslationCommand(drivebase,1.0,0.0).withTimeout(4.0)
        );


        // ============ CONDUÇÃO LENTA PELO BOTÃO LB ============
        // Enquanto LB estiver pressionado, utiliza a condução com translação reduzida.
        driverXbox.leftBumper().whileTrue(
                drivebase.driveFieldOriented(driveLowSpeed)
        );
    }


    // ============ CONFIGURAÇÃO DO AUTÔNOMO ============
    // Registra os comandos nomeados e cria o seletor de autônomo no dashboard.
    // Os comandos nomeados precisam ser registrados ANTES de criar o seletor.
    private void configureAutonomous() {

        // ============ COMANDOS NOMEADOS ============
        // Permitem usar ações do robô dentro dos autos do PathPlanner, pelo nome.
        // "test" só imprime no console; os comandos de intake e elevador entram aqui depois.
        NamedCommands.registerCommand("test", Commands.print("Comando nomeado 'test' executado"));

        // ============ SELETOR NO DASHBOARD ============
        // Se o AutoBuilder foi configurado, monta o seletor com os autos do PathPlanner.
        // Se não (por exemplo, settings.json ausente), mostra uma opção "Nenhum" para não quebrar.
        if (AutoBuilder.isConfigured()) {
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption("Nenhum (PathPlanner nao configurado)", Commands.none());
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }


    // ============ COMANDO AUTÔNOMO ============
    // Retorna o auto escolhido no seletor do dashboard.
    // Chamado pelo Robot.java ao iniciar o período autônomo.
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


    // ============ FREIO DOS MOTORES ============
    // Define se os motores de movimentação devem permanecer em modo de frenagem.
    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}