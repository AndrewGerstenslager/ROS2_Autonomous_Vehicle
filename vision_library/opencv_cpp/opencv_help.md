# Need to install OpenCV for your local machine? 
## Try out this!
### Note: I am writing this for Ubuntu 20.04
### Note: This is also for VS Code

### 0. Update and install packages

sudo apt update && sudo apt upgrade
sudo apt install build-essential gdb

### 1. Check if OpenCV is installed

run "pkg-config --modversion opencv4"

### 2. Install c++ and other related packages in VS Code

### 3. Add opencv to the c++ extension

1. In Visual Studio Code, open the settings (File > Preferences > Settings or press Ctrl+,).

2. Click on the "Extensions" tab in the settings window, then scroll down and find "C/C++" and click on "Edit in settings.json" under "C_Cpp > Default > Include Path."

3. You should see a JSON file named settings.json. If there is no "settings.json" file, create one in the .vscode folder inside your project folder.

4. In settings.json, add the following configuration, which tells the C++ extension where to find the OpenCV header files:

'''json
{
    "C_Cpp.default.includePath": [
        "${workspaceFolder}/**",
        "/usr/local/include/opencv4"
    ]
}
'''


NOTE: The GUI for settings also has a button where you can type in and add the path for opencv.
