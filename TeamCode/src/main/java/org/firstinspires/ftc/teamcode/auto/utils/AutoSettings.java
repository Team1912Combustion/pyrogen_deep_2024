
package org.firstinspires.ftc.teamcode.auto.utils;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import java.io.File;

public class AutoSettings {

    private String filename = "AutoConfig.json";
    private AutoConfig myAutoConfig = new AutoConfig();

    public boolean I_AM_BLUE = false;
    public boolean GO_SAMPLE = true;
    public boolean GO_CLIMB = true;
    public boolean GO_PARK = true;
    public boolean BACKSTAGE = false;

    public boolean iAmBlue() { return I_AM_BLUE; }
    public boolean goSample() { return GO_SAMPLE; }
    public boolean goClimb() { return GO_CLIMB; }
    public boolean goPark() { return GO_PARK; }
    public boolean iAmBackstage() { return BACKSTAGE; }

    public void pushToAutoConfig () {
        myAutoConfig.I_AM_BLUE = I_AM_BLUE;
        myAutoConfig.GO_SAMPLE = GO_SAMPLE;
        myAutoConfig.GO_CLIMB = GO_CLIMB;
        myAutoConfig.GO_PARK = GO_PARK;
        myAutoConfig.BACKSTAGE = BACKSTAGE;
    }

    public void pullFromAutoConfig () {
        I_AM_BLUE     = myAutoConfig.I_AM_BLUE;
        GO_SAMPLE     = myAutoConfig.GO_SAMPLE;
        GO_CLIMB      = myAutoConfig.GO_CLIMB;
        GO_PARK       = myAutoConfig.GO_PARK;
        BACKSTAGE     = myAutoConfig.BACKSTAGE;
    }
    public void copyToAutoConfig(AutoConfig tmpAutoConfig) {
        myAutoConfig.I_AM_BLUE     = tmpAutoConfig.I_AM_BLUE;
        myAutoConfig.GO_SAMPLE     = tmpAutoConfig.GO_SAMPLE;
        myAutoConfig.GO_CLIMB      = tmpAutoConfig.GO_CLIMB;
        myAutoConfig.GO_PARK       = tmpAutoConfig.GO_PARK;
        myAutoConfig.BACKSTAGE     = tmpAutoConfig.BACKSTAGE;
    }
    public void saveAutoConfig() {
        pushToAutoConfig();
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, myAutoConfig.serialize());
    }
    public void readAutoConfig() {
        File file = AppUtil.getInstance().getSettingsFile(filename);
        try {
            AutoConfig tmpAutoConfig = AutoConfig.deserialize(ReadWriteFile.readFile(file));
            copyToAutoConfig(tmpAutoConfig);
        } catch (Exception e) { // if the read fails, presumably the file is not there to create it
            saveAutoConfig();
        }
        pullFromAutoConfig();
    }
}

class AutoConfig implements Cloneable
{
    public boolean I_AM_BLUE = true;
    public boolean GO_SAMPLE = true;
    public boolean GO_CLIMB = true;
    public boolean GO_PARK = true;
    public boolean BACKSTAGE = true;
    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static AutoConfig deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, AutoConfig.class);
    }
    public AutoConfig clone()
    {
        try {
            AutoConfig result = (AutoConfig) super.clone();
            return result;
        }
        catch (CloneNotSupportedException e)
        {
            throw new RuntimeException("internal error: AutoConfig can't be cloned");
        }
    }
}
