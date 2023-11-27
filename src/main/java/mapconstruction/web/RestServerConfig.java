package mapconstruction.web;

import com.fasterxml.jackson.annotation.JsonProperty;
import io.dropwizard.Configuration;
import org.hibernate.validator.constraints.NotEmpty;

/**
 * Obligated class. Doesn't contain anything interesting..
 *
 * @author Jorrick
 * @since 18/09/2018
 */
public class RestServerConfig extends Configuration {
    @NotEmpty
    private String version;

    @JsonProperty
    public String getVersion() {
        return version;
    }

    @JsonProperty
    public void setVersion(String version) {
        this.version = version;
    }
}