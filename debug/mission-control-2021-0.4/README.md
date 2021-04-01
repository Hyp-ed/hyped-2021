# Mission Control 2021

This is the repository for the HYPED Mission Control, which is responsible for communicating with the hyperloop pod, sending relevant commands to it and displaying received information in the GUI. Backend is written in Java with the Spring Boot framework, it communicates to the pod using JSON messages sent over a TCP socket, and displays information on the React frontend.

### How to run
First of all, make sure that your Java version is 8 or up.

Then download the latest release from [Github](https://github.com/Hyp-ed/mission-control-2021/releases), put it inside your `hyped-2021` repo if you want the debug functionality,  and run:
```
$ java -jar mission-control-2021.jar
```

Go to `localhost:8080` for the GUI.

### How to build the project:
This project uses gradle as its build system. Only manually build the project instead of using the released jar file as explained above if you actually intend to work on the mission control. The gradle wrapper is already checked into this repo, so no need to explicitly download gradle (unless you want to).

1. Download and install nodeJS from [nodejs.org](https://nodejs.org/en/) \
   You can check whether it was successfully installed by typing `node -v && npm -v` in the terminal.

2. Install yarn: `sudo npm install yarn -g`

3. Clone this repo

4. Run to build: `./gradlew build`. This will create a jar file in `build/libs/` that contains both the backend and the static frontend that it serves.

5. Launch Mission Control: `java -jar build/libs/mission-control-2021.jar`

6. Go to `localhost:8080` for the GUI.
