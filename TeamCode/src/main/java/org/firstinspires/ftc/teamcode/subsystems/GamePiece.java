
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;

public class GamePiece extends SubsystemBase {
    private enum gamePiece {
        SAMPLE,
        SPECIMEN
    };

    gamePiece type;

    public GamePiece() {
        type = gamePiece.SAMPLE;
    }

    public boolean sample() {
        return type == gamePiece.SAMPLE;
    }
    public void toggle() {
        if (type == gamePiece.SPECIMEN) {
            type = gamePiece.SAMPLE;
        } else {
            type = gamePiece.SPECIMEN;
        }
    }
}
