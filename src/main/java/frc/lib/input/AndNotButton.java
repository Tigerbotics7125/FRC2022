package frc.lib.input;

import edu.wpi.first.wpilibj2.command.button.Button;

/** @author Spectrum 3847 */
public class AndNotButton extends Button {

  Button b1;
  Button b2;

  public AndNotButton(Button button, Button button2) {
    b1 = button;
    b2 = button2;
  }

  public boolean get() {
    return b1.get() && !b2.get();
  }
}
