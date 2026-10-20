
package frc.robot.commands.swervedrive.drivebase;

// ============ IMPORTAÇÕES ============
// Importa os controladores PID, a geometria, as velocidades e a telemetria.
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.swervedrive.SwerveSubsystem;


// ============ CLASSE DE DESLOCAMENTO RELATIVO ============
// Move o robô para uma posição calculada a partir da pose registrada no início do comando.
public class DriveRelativeTranslationCommand extends Command {

    // ============ ATRIBUTOS DE MOVIMENTAÇÃO ============ 
    // Armazena o subsistema, o deslocamento desejado e os limites de velocidade.
    private final SwerveSubsystem swerve;
    private final double deltaX;
    private final double deltaY;
    private final double maxLinearSpeed;
    private final double maxAngularSpeed;


    // ============ CONTROLADORES PID ============ 
    // Controla os erros de posição nos eixos X e Y e o erro de orientação.
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;


    // ============ ALVOS E TOLERÂNCIAS ============ 
    // Define a pose de destino e os limites usados para considerar o movimento concluído.
    private Pose2d targetPose;
    private Rotation2d targetHeading;

    private static final double POSITION_TOLERANCE_METERS = 0.02;
    private static final double HEADING_TOLERANCE_RADIANS =
            Units.degreesToRadians(1.0);


    // ============ ESTABILIDADE DA POSIÇÃO ============ 
    // Exige que o robô permaneça dentro das tolerâncias por vários ciclos consecutivos.
    private int withinToleranceCount = 0;
    private static final int REQUIRED_STABLE_LOOPS = 5;


    // ============ CONSTRUTOR COMPLETO ============ 
    // Recebe o deslocamento relativo, as velocidades máximas e configura os controladores.
    public DriveRelativeTranslationCommand(
            SwerveSubsystem swerve,
            double deltaX,
            double deltaY,
            double maxLinearSpeed,
            double maxAngularSpeed) {

        this.swerve = swerve;
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        this.maxLinearSpeed = maxLinearSpeed;
        this.maxAngularSpeed = maxAngularSpeed;

        xController = new PIDController(2.0, 0.0001, 0.0);
        yController = new PIDController(2.0, 0.0001, 0.0);
        headingController = new PIDController(3.0, 0.0, 0.0);

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(POSITION_TOLERANCE_METERS);
        yController.setTolerance(POSITION_TOLERANCE_METERS);
        headingController.setTolerance(HEADING_TOLERANCE_RADIANS);

        addRequirements(swerve);
    }


    // ============ CONSTRUTOR SIMPLIFICADO ============ 
    // Permite definir apenas o deslocamento e utiliza limites de velocidade padrão.
    public DriveRelativeTranslationCommand(
            SwerveSubsystem swerve,
            double deltaX,
            double deltaY) {

        this(swerve, deltaX, deltaY, 1.5, Math.PI);
    }


    // ============ INICIALIZAÇÃO DO DESLOCAMENTO ============ 
    // Registra a pose inicial e calcula o alvo relativo à orientação do robô naquele instante.
    @Override
    public void initialize() {

        Pose2d initialPose = swerve.getPose();

        // ============ VALIDAÇÃO DA POSE INICIAL ============ 
        // Usa a origem como alternativa caso as coordenadas retornadas sejam inválidas.
        if (Double.isNaN(initialPose.getX())
                || Double.isNaN(initialPose.getY())) {

            initialPose = new Pose2d();
        }


        // ============ CÁLCULO DA POSE DE DESTINO ============ 
        // Gira o deslocamento relativo para o referencial do campo e soma à posição inicial.
        targetHeading = initialPose.getRotation();

        Translation2d relativeTranslation =
                new Translation2d(deltaX, deltaY);

        Translation2d fieldTranslation =
                relativeTranslation.rotateBy(targetHeading);

        targetPose = new Pose2d(
                initialPose.getTranslation().plus(fieldTranslation),
                targetHeading
        );


        // ============ REINICIALIZAÇÃO DOS CONTROLADORES ============ 
        // Zera o estado interno dos PID e o contador de estabilidade.
        xController.reset();
        yController.reset();
        headingController.reset();

        withinToleranceCount = 0;
    }


    // ============ EXECUÇÃO DO DESLOCAMENTO ============ 
    // Calcula as correções necessárias para aproximar a pose atual da pose de destino.
    @Override
    public void execute() {

        Pose2d currentPose = swerve.getPose();


        // ============ TELEMETRIA DA ODOMETRIA ============ 
        // Publica a posição atual, o alvo e os erros para acompanhamento no SmartDashboard.
        SmartDashboard.putNumber(
                "Odometria/X Atual",
                currentPose.getX()
        );

        SmartDashboard.putNumber(
                "Odometria/Y Atual",
                currentPose.getY()
        );

        SmartDashboard.putNumber(
                "Alvo/X",
                targetPose.getX()
        );

        SmartDashboard.putNumber(
                "Alvo/Y",
                targetPose.getY()
        );

        SmartDashboard.putNumber(
                "Erro/X",
                targetPose.getX() - currentPose.getX()
        );

        SmartDashboard.putNumber(
                "Erro/Y",
                targetPose.getY() - currentPose.getY()
        );

        SmartDashboard.putNumber(
                "Odometria/Heading",
                currentPose.getRotation().getDegrees()
        );

        SmartDashboard.putNumber(
                "Alvo/Heading",
                targetHeading.getDegrees()
        );


        // ============ CÁLCULO DAS CORREÇÕES PID ============ 
        // Calcula as velocidades de correção dos eixos e da orientação.
        double xSpeed = xController.calculate(
                currentPose.getX(),
                targetPose.getX()
        );

        double ySpeed = yController.calculate(
                currentPose.getY(),
                targetPose.getY()
        );

        double rotSpeed = headingController.calculate(
                currentPose.getRotation().getRadians(),
                targetHeading.getRadians()
        );


        // ============ LIMITAÇÃO DAS VELOCIDADES ============ 
        // Impede que as velocidades calculadas ultrapassem os limites configurados.
        xSpeed = Math.max(
                -maxLinearSpeed,
                Math.min(maxLinearSpeed, xSpeed)
        );

        ySpeed = Math.max(
                -maxLinearSpeed,
                Math.min(maxLinearSpeed, ySpeed)
        );

        rotSpeed = Math.max(
                -maxAngularSpeed,
                Math.min(maxAngularSpeed, rotSpeed)
        );


        // ============ CONVERSÃO PARA VELOCIDADES DO ROBÔ ============ 
        // Converte as velocidades do referencial do campo para o referencial do robô.
        ChassisSpeeds targetChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        rotSpeed,
                        currentPose.getRotation()
                );


        // ============ EXECUÇÃO DA MOVIMENTAÇÃO ============ 
        // Envia as velocidades calculadas para o subsistema swerve.
        swerve.drive(targetChassisSpeeds);


        // ============ VERIFICAÇÃO DAS TOLERÂNCIAS ============ 
        // Conta os ciclos consecutivos em que posição e orientação estão dentro dos limites.
        boolean atX = xController.atSetpoint();
        boolean atY = yController.atSetpoint();
        boolean atHeading = headingController.atSetpoint();

        if (atX && atY && atHeading) {
            withinToleranceCount++;
        } else {
            withinToleranceCount = 0;
        }
    }


    // ============ VERIFICAÇÃO DE CONCLUSÃO ============ 
    // Finaliza após atingir as tolerâncias durante a quantidade de ciclos exigida.
    @Override
    public boolean isFinished() {
        return withinToleranceCount >= REQUIRED_STABLE_LOOPS;
    }


    // ============ FINALIZAÇÃO DO DESLOCAMENTO ============ 
    // Interrompe o movimento ao concluir o deslocamento ou quando o comando é cancelado.
    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
}
