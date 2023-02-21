package frc.robot;

import java.util.HashMap;

import javax.security.auth.kerberos.EncryptionKey;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;


public class TabData {
    ShuffleboardTab tab;
    HashMap<String, GenericEntry> entries;
    int defaultValue = 0;

    public TabData(String tabName){
        tab = Shuffleboard.getTab(tabName);

        entries = new HashMap<String, GenericEntry>();
    }
    
    public GenericEntry getEntry(String entryname){
        if(entries.containsKey(entryname)){
            return entries.get(entryname);
        }else{
            GenericEntry entry = tab.add(entryname, defaultValue).getEntry();
            entries.put(entryname, entry);
            return entry;
        }
    }

    public void updateEntry(String entryname, Object value){
        GenericEntry entry = getEntry(entryname);
        if(String.class.equals(value.getClass())){
            entry.setString((String) value);
        }else{
            entry.setValue(value);
        }
    }
}