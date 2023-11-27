package mapconstruction.web.config;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.io.Serializable;

import static java.lang.String.format;

public class DatasetConfig implements Serializable {
    private String system;
    private double zone;
    private String hemisphere;
    private String path;
    private boolean isWalkingDataset;

    @JsonProperty
    public String getSystem() {
        return system;
    }

    public void setSystem(String system) {
        this.system = system;
    }

    @JsonProperty
    public double getZone() {
        return zone;
    }

    public void setZone(double zone) {
        this.zone = zone;
    }

    @JsonProperty
    public String getHemisphere() {
        return hemisphere;
    }

    public void setHemisphere(String hemisphere) {
        this.hemisphere = hemisphere;
    }

    @JsonProperty
    public String getPath() {
        return path;
    }

    public void setPath(String path) {
        this.path = path;
    }

    @JsonIgnore
    public void setWalkingDataset(boolean walking){
        isWalkingDataset = walking;
    }

    @JsonProperty
    public boolean isWalkingDataset(){
        return isWalkingDataset;
    }


    @Override
    public String toString() {
        return format("System : %s\n", system) +
                format("Zone : %s\n", zone) +
                format("Hemisphere : %s\n", hemisphere) +
                format("Path : %s\n", path);
    }
}
