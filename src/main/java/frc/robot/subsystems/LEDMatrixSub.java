package frc.robot.subsystems;

import frc.tigerlib.ledmatrix.Matrix2D;
import static frc.robot.constants.LEDMatrixConstants.*;

public class LEDMatrixSub {
    private Matrix2D m_matrix;

    public LEDMatrixSub() {
        try {
            m_matrix = new Matrix2D(kWidth, kHeight, kSerpentine, kPWMPort, kGifs);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
