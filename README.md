# 2024_Crescendo
Welcome to the LambdaCorps code for the FRC 2024 season. The start of the repository should be a robot that can simulate properly, drive forward with positive movements, and properly turn left and right.

To run the simulator without any code changes:
```python -m robotpy sim```

# System Preparation
These are the steps needed to follow to develop code for the season.  

The following command lines are being run from the MacOS Terminal. If you are running these steps in Windows you'll need to replace ```python``` with ```py -3```.

## Computer Installation
Each system developing, simulating, and deploying code to the robot needs to have the robotpy installation completed.  The steps for computer setup are found at the [WPILIB Documentation Site](https://frcdocs.wpi.edu/en/latest/docs/zero-to-robot/step-2/python-setup.html).

The most important thing is to install Robotpy, and then install the extras with sync.
1. Install Robotpy
    ```pip install robotpy```
1. Install the extras by running robotpy sync
    ```python -m robotpy sync```
    * NOTE: This step may currently fail (as of 10 January 2024) due to the commands framework not being officially released yet. If it fails, just comment (put a # in front of) the commands 2 line and re-run the ___sync___ command above.
    * If not released yet, you can manually install the beta version of commands 2 with Pip
        ```pip install robotpy-commands-v2==2024.0.0b4```

## Repository Creation
Because you're following the steps from this repository, the basic python project has already been setup for you. The steps followed to create this repository were:
1. Initialize an empty directory with Robotpy
    ```python -m robotpy init```
1. Edit the resulting _pyroject.toml_ file to add the extras we're using.
    ```
    #
    # Use this configuration file to control what RobotPy packages are installed
    # on your RoboRIO
    #

    [tool.robotpy]

    # Version of robotpy this project depends on
    robotpy_version = "2024.1.1.2"

    # Which extra RobotPy components should be installed
    # -> equivalent to `pip install robotpy[extra1, ...]
    robotpy_extras = [
        # "all"
        "apriltag",                # <-- Uncommented this line, add a comma ','
        "commands2",               # <-- Uncommented this line, add a comma ','
        # "cscore"
        "navx",                    # <-- Uncommented this line, add a comma ','
        # "pathplannerlib"
        # "phoenix5"
        "phoenix6",                # <-- Uncommented this line, add a comma ','
        # "playingwithfusion"
        # "rev"
        # "romi"
        "sim",                     # <-- Uncommented this line, add a comma ','
        # "xrp"
    ]

    # Other pip packages to install
    requires = []
    ```
1. Install the extras by running robotpy sync
    ```python -m robotpy sync```
    * NOTE: This step may currently fail (as of 10 January 2024) due to the commands framework not being officially released yet. If it fails, just comment (put a # in front of) the commands 2 line and re-run the ___sync___ command above.
    * If not released yet, you can manually install the beta version of commands 2 with Pip
        ```pip install robotpy-commands-v2==2024.0.0b4```
1. Create the physics.py file for our simulation
    ```python -m robotpy create-physics```
1. Edit the _robot.py_ file to have an actual robot in code.

# Troubleshooting
We saw on some of the laptops that updating Robotpy didn't work well.  The only way to get the new version installed correctly was to remove all the old related robotpy packages, and uninstall them.  If you need to remove the robotpy related Python packages and reinstall them try the following command in the laptop's terminal:
```
$ for i in `pip list | grep robotpy | cut -d ' ' -f1` ; do pip uninstall -y $i ; done
```
Then, reinstall robotpy:
```
$ pip install robotpy
 ```