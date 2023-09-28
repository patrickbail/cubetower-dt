# Running standalone Isaac Sim Python scripts
### Working with the Core API in Visual Studio Code

Every standalone Python script that makes use of the Omniverse API, needs to be inside the Isaac Sim directory to get IntelliSense and Autocomplete while working on it.

More details about the pre-configured enviroment can be looked up at the [Omniverse Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_standalone_python.html)
### Launching a standalone Isaac Sim Python script
If Assets need to be loaded from the Omniverse API, then an access to a Nucleus database (either a connection to the local Nucleus service or a remote one) has to be established (can be done by starting the Omniverse Launcher). Only then Assets, like a ground plane and more, can be found and loaded into the simulation enviroment.

Run a standalone Isaac Sim Python script via:
* Linux `./python.sh <path_to_python_script>`
* Windows `./python.bat <path_to_python_script>`

To test if it works, try running: 
* Linux
`./python.sh ./standalone_examples/api/omni.isaac.core/simulate_robot.py`
* Windows
`./python.bat .\standalone_examples\api\omni.isaac.core\simulate_robot.py`