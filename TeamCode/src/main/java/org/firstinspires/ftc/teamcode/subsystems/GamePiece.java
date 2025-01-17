
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

    public boolean is_sample() {
        return type == gamePiece.SAMPLE;
    }
    public void sample() {
        type = gamePiece.SAMPLE;
    }
    public void specimen() {
        type = gamePiece.SPECIMEN;
    }
    public void toggle() {
        if (type == gamePiece.SPECIMEN) {
            type = gamePiece.SAMPLE;
        } else {
            type = gamePiece.SPECIMEN;
        }
    }
}
