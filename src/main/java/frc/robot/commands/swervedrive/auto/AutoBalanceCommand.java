
package frc.robot.commands.swervedrive.auto;

// ============ IMPORTAÇÕES ============
// Importa as ferramentas de controle PID, movimentação e telemetria.
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.swervedrive.SwerveSubsystem;


// ============ CLASSE DE AUTO BALANCE ============
// Utiliza o ângulo de inclinação do robô para tentar nivelá-lo.
public class AutoBalanceCommand extends Command {

    // ============ ATRIBUTOS ============ 
    // Armazena o subsistema de movimentação e o controlador PID.
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController controller;


    // ============ CONSTRUTOR ============ 
    // Configura o PID e declara que este comando utiliza o swerve.
    public AutoBalanceCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        controller = new PIDController(1.0, 0.0, 0.0);
        controller.setTolerance(1);
        controller.setSetpoint(0.0);

        addRequirements(this.swerveSubsystem);
    }


    // ============ INICIALIZAÇÃO ============ 
    // Executado uma vez quando o comando é iniciado.
    @Override
    public void initialize() {
    }


    // ============ EXECUÇÃO DO BALANCEAMENTO ============ 
    // Calcula a correção pela inclinação e limita a velocidade de movimentação.
    @Override
    public void execute() {
        SmartDashboard.putBoolean(
                "At Tolerance",
                controller.atSetpoint()
        );

        double translationVal = MathUtil.clamp(
                controller.calculate(
                        swerveSubsystem.getPitch().getDegrees(),
                        0.0
                ),
                -0.5,
                0.5
        );

        swerveSubsystem.drive(
                new Translation2d(translationVal, 0.0),
                0.0,
                true
        );
    }


    // ============ VERIFICAÇÃO DE CONCLUSÃO ============ 
    // Encerra o comando quando o PID considera que o robô está nivelado.
    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }


    // ============ FINALIZAÇÃO ============ 
    // Bloqueia os módulos quando o comando termina ou é interrompido.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.lock();
    }
}
