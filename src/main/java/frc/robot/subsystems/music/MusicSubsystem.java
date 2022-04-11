package frc.robot.subsystems.music;

import java.util.Collection;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MusicSubsystem extends SubsystemBase {
    
    private Logger logger = Logger.getLogger("Subsystem Factory");

    private Map<String, String> songs;
    private Orchestra orchestra;
    private ShuffleboardTab musicTab = Shuffleboard.getTab("Music");
    private NetworkTableEntry currentSong = musicTab.add("Current Song", "None").getEntry();
    private NetworkTableEntry reloadButton = musicTab.add("Reload", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private NetworkTableEntry playButton = musicTab.add("Play", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private NetworkTableEntry stopButton = musicTab.add("Stop", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private NetworkTableEntry pauseButton = musicTab.add("Pause", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private NetworkTableEntry songTimeStamp = musicTab.add("Song Timestamp", 0.0).getEntry();

    /**
     * Constructor without songs
     * @param musicMotors Collection of TalonFXs that can play music
     */
    public MusicSubsystem(Collection<TalonFX> musicMotors) {
        orchestra = new Orchestra(musicMotors);
    }

    /**
     * Constructor with songs
     * @param musicMotors Collection of TalonFXs that can play music
     * @param songs A map of songs. First string should be song name and second string should be song path for each entry.
     */
    public MusicSubsystem(Collection<TalonFX> musicMotors, Map<String, String> songs) {
        orchestra = new Orchestra(musicMotors);
        this.songs = songs;
    }

    public void init() {
        songTimeStamp.forceSetNumber(orchestra.getCurrentTime());
        if (reloadButton.getBoolean(false)) {
            loadSong(currentSong.getString(""));
            reloadButton.forceSetBoolean(false);
        }
        if (playButton.getBoolean(false)) {
            play();
            playButton.forceSetBoolean(false);
        }
        if (stopButton.getBoolean(false)) {
            stop();
            stopButton.forceSetBoolean(false);
        }
        if (pauseButton.getBoolean(false)) {
            pause();
            pauseButton.forceSetBoolean(false);
        }
    }

    public void addSong(String songName, String songPath) {
        songs.put(songName, songPath);
    }

    public void removeSong(String songName) {
        songs.remove(songName);
    }

    public boolean loadSong(String songName) {
        if (!songs.containsKey(songName)) {
            logger.log(Level.WARNING, "Song Name unrecognized");
            return false;
        }
        orchestra.loadMusic(songs.get(songName));
        currentSong.forceSetString(songName);
        return true;
    }

    public void play() {
        orchestra.play();
    }

    public void stop() {
        orchestra.stop();
    }

    public void pause() {
        orchestra.pause();
    }

    public void reloadSong() {
        orchestra.loadMusic(currentSong.getString(""));
    }

    public boolean isPlaying() {
        return orchestra.isPlaying();
    }

    public void addMotor(TalonFX newMotor) {
        orchestra.addInstrument(newMotor);
    }
}
