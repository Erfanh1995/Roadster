# Bundle-based Map Construction

Map construction has always been a very expensive task.
In an effort to reduce these costs we are working on creating a traversable network using a form of Fréchet distance.
The original paper by Kevin Buchin et Al. illustrates the idea of map construction using Fréchet distance.
Roel Jacobs implemented this paper in his master thesis. Here he managed to implement the bundling and create a final road map.
The goal of this paper, and thus repository, is to extend upon the research of Roel Jacobs.

## Getting Started

Clone this project in your favorite IDE (Intellij IDEA recommended).

### Prerequisites

The IDEA has to have support for MAVEN dependency management.
First you have to right click on the pom.xml file and import it as a maven file.
This will automatically offer you to download the rest of the dependencies.
If it does not have support for MAVEN, you have to install MAVEN yourself.

### Setting up the config files
For the application to be able to run we have to create two files in the project directory.

The first file is the *web-config.yml*. This contains the config for the webserver.
This file is also included in the repository and does not need any modifications. It looks as follows.
```
version: 0.0.1

server:
  rootPath: /api/
  applicationConnectors:
  - type: http
    port: 9000
```

The second file is the *general-config.yml*. This file contains more general configurations of the program.
The port should be the same as the port in *web-config.yml*. Furthermore, you specify whether you want to open the
web page when we start the program. The Google Maps API key should be supplied here as well. Finally we define the
locations of the dataset and saved state directory. It is best to exactly copy this and put it in a file (and just change the googleMapsApiKey).
```
webPagePort: 9000
openWebPageOnStart: false
googleMapsApiKey: AIzaSyCbkVz1YC36rd_Q-7-iSQLbRBIKkI3qylU
datasetDirectory: /data/
savedStatesDirectory: /savst/
```

### Web run configurations

The application can be run multiple ways. You can either use the web interface or start the computations by console.
In order to comply with the standards of a sequential jobs used on the supercomputer of University of Sydney, another starting interface will be created.
This interface will allow for starting the calculation of all bundles, their properties and possibly the final map.

To start the current application, we create a run configuration with the following properties:
```
Main class: mapconstruction.web.RestServerApp
Program arguments: server web-config.yml
Working directory: current directory
JRE: 1.8
Before execution: Run maven compile
```

And the maven compile run configuration should just be a maven run configuration:
```
Working directory: current directory
Command line: compile -f pom.xml until finished
```

### Running on datasets

The datasets used at the moment of writing are the datasets made available by [mapconstruction.org](http://mapconstruction.org/).
Datasets, however, do not work out of the box.
A dataset folder called *data* should be next to our src folder.
In this folder, the txt files should be located.
Next to all txt files, we also have to specify the following in *dataset-config.yml*:
- The geographic system used for the coordinates.
- The zone of the coordinates.
- The hemisphere of the coordinates.
In most cases, this would mean that our *dataset-config.yml* in our dataset folder would look like
```
system: UTM
zone: 16
hemisphere: N
```
The zone and hemisphere could be different for every city but the system would be the same for most.
However, not for the Athens dataset. This one uses a slightly different system and hence our *dataset-config.yml* file looks like
```
system: GGRS87
zone: 34.5
hemisphere: N
```
As road maps tend to change over time, the [mapconstruction.org](http://mapconstruction.org/) organization also supplied the roadmap at that time.
You can add the *edges.txt* and *vertices.txt* files in a subfolder of the dataset called *verification*.
This will allow you to draw this network as a background in the web interface.

### Running on saved states

Every time a computation of one of the larger algorithm is finished, we store the current state of the controller.
This state is called a saved state.
The saved states are stored in a folder next to our src folder called *savst*.
The name of the saved state will be equal to the exact time when the algorithm finished.

One is allowed to change the name of the saved state.
However, it is not recommended to make changes to the dataset (folder) used for this saved state.
The trajectories will not be required anymore but the verification of the dataset will be loaded on-demand and hence are not stored in the saved state.
Thus changing the dataset folder name would result in errors when trying to get the verification.


### Terminal based run configuration
The terminal run configuration is set by starting mapconstruction.starter.Starter.
There are several arguments that have to be added.
First of all, you have to specify the configPath relative to project directory. You could use 'artemis-config.yml' here.
Second, we either specify the savedState or the dataset to be used.
Finally, we specify what we want to compute.
An example of a run configuration can be found in the next chapter, Artemis run configuration.
The commands are as follows:
```
usage: Starter.java
 -p,--configPath <arg>       General config path
 -s,--savedState <arg>       SavedState input file
 -d,--dataset <arg>          Dataset input directory
 -call,--computeAll          Compute all
 -crn,--computeRoadNetwork   Compute road network
 -cb,--computeBundles        Compute bundles
```

### Artemis run configuration
Artemis, the super computer at Sydney University, uses a terminal program for queuing it's jobs.
This requires us to specify our research project's name, BBMC, the number of cores and memory we want, and the walltime, the time it can take at most.
The following code allows our code to run. To compile as well, uncomment the line of mvn package.

```
!/bin/bash
#PBS -P BBMC
#PBS -l select=1:ncpus=4:mem=4GB
#PBS -l walltime=00:15:00

#cd "$PBS_O_WORKDIR"
cd /home/jsle3577/JavaMapConstruction
dt=$(date '+%Y-%m-%d-%H.%M.%S')
results_file="results/${dt}results.out"

module load maven/3.5.2
# mvn package
java -cp target/MapConstructionWeb-1.0-SNAPSHOT.jar mapconstruction.starter.Starter -p artemis-config.yml -d athens_xxsmall -call > $results_file
```
Storing this file as a .pbs file and running 'qsub filename.pbs' will queue the task.
To view the update of the program, run 'watch -n 1 jobstat'

## Running the tests

In the folder 'src/tests' there are several tests that test small parts of the program.
This can be run using your IDE or by instantiating the JUnit test framework.

## Development

The original application was developed by Roel Jacobs.
This application was built using a javafx interface.
As using web based technologies could speed up development,
it was decided that this interface would be removed and replaced by a web interface.
As a result, there are several classes remaining which are not used anymore and some functionality might not be possible anymore.
Furthermore a lot of configuration options regarding the bundling are still possible but also not implemented into this interface.

### General coding comments
Every api call which returns a JSON uses Jackson(one of the packages of dropwizard) to convert Java objects into JSON.
It does play well with almost all objects except for Line2D. There is a self reference in Line2D causing an infinite loop.
As there seems to be no short time solution for the problem, I decided to convert all Line2D's we would normally JSONize, into Point2D's.

As Jackson can not convert private objects, we have to create getters for Jackson.
For this reason, some files, the Bundle.java file for example, contains a lot of getters ending with JSON.
These getters are not supposed to be used during the program and are only there for debugging.
This way we allow ourselves to easily turn on debugging, and thus return a bunch of extra values, by changing one boolean variable to true.
However, the JSON formatter can only handle so many values. Therefore, we try to prevent reporting values that are not required when continuing with other parts.

## Built With

* [Dropwizard](http://www.dropwizard.io/1.3.5/docs/) - The web framework used
* [SnakeYAML](https://bitbucket.org/asomov/snakeyaml/wiki/Documentation) - The data serialization used for config files
* [JUnit](https://junit.org/junit5/) - The testing framework used
* [Maven](https://maven.apache.org/) - Dependency Management


## Authors

* **Roel Jacobs** - *Initial work* - Paper of his research is available on [ACM](https://dl.acm.org/citation.cfm?id=3139958.3139964)
* **Jorrick Sleijster** - *Further research* - Master Thesis
* **Jorren Hendricks** - *Further research* - Master Thesis
* **Erfan Hosseini Sereshgi - *Further research*

## License

This project is property of the TU/e and Tulane University.

## Acknowledgments

* **Kevin Buchin** as supervisor of Roel Jacobs, Jorrick Sleijster and Jorren Hendricks
* **Joachim Gudmundsson** as second supervisor of Jorrick Sleijster
* **Carola Wenk** as supervisor of Erfan Hosseini Sereshgi
