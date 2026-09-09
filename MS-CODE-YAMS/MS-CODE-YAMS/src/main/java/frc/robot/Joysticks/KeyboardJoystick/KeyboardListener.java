package frc.robot.Joysticks.KeyboardJoystick;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.BooleanPublisher;

public class KeyboardListener implements KeyListener {
    // Instancia os publicadores das NetworkTables
    private BooleanPublisher garraBotaoPub;

    public KeyboardListener() {
        // Inicializa o NetworkTables apontando para a tabela "SmartDashboard"
        var table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        garraBotaoPub = table.getBooleanTopic("Teclado/AbrirGarra").publish();
    }

    @Override
    public void keyPressed(KeyEvent e) {
        // Se pressionar a tecla 'A', avisa o robô (true)
        if (e.getKeyCode() == KeyEvent.VK_A) {
            garraBotaoPub.set(true);
        }
    }

    @Override
    public void keyReleased(KeyEvent e) {
        // Se soltar a tecla 'A', desliga a função no robô (false)
        if (e.getKeyCode() == KeyEvent.VK_A) {
            garraBotaoPub.set(false);
        }
    }

    @Override
    public void keyTyped(KeyEvent e) {}
}
