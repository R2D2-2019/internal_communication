
# Build workflow
The build workflow for R2D2-2019 is based on the BMPTK (bare metal programming tool kit) made by Wouter van Ooijen. This guide will help you install and work with this environment. If you are experiencing any problems installing or working with the environment you can contact the leads, note for windows it is best to contact Gerrit, for linux it is best to contact Lex or Patrick. All of us are able to help you most of the time though.  

## Installation
 
 ### Windows:

 1. Install git by visiting the git [download](https://git-scm.com/download/win) page.  Install using all the default settings and make sure the checkbox which adds git to your path is checked. **note:** If you already installed git on your machine you can skip this step.
 2. Pull the build repo by typing `git clone  https://github.com/R2D2-2019/R2D2-build` you can clone repo to whatever drive or folder you want. Remember the path you cloned the repo to.
 3. Open windows explorer and browse to the path you cloned the repo to, browse to the folder `R2D2-build/programs/install`
 4. Double-click on the "mingw-w64-install.exe" after the installation wizard has opened click next until you can select the installation path at this point change the path to the following `<path-to-your-repo>/R2D2-build/programs`. Install by clicking next. After installation a folder `mingw32` should have appeared int the `R2D2-build/programs` directory.
 5.  Double-click on the "gcc-arm-none-eabi-8-2018-q4-major-win32-sha1.exe" after the installation wizard has opened click next until you can select the installation path at this point change the path to the following `<path-to-your-repo>/R2D2-build/programs`. Install by clicking next. After installation a folder `8 2018-q4-major` should have appeared in the `R2D2-build/programs` directory.
 6. Return to the main directory of the repository and run the "init.bat" file. **note:** this file needs to be run as administrator and add all the necessary components to the path environment variable.<br> If you prefer to do this manually you are welcome to do so. The most important path to add is the following `<path-to-your-repo>/R2D2-build/programs/bmptk/tools`
 7. Check if everything is working by opening a command prompt en typ `bmptk-make` if the console returns something like this `bmptk-make: *** no targets specified and no makefiles found. Stop.` everything should work fine.

**Warning:** This installation replaces earlier installations of BMPTK make sure to remove the old installation from your PATH environment variable's.  

### Linux:
The installation on linux is quite simple below a list of packages you need to install, this can be achieved by using your own package manager. Replace apt-get with your own package manager. Most packages are probably already installed.

-   `sudo apt-get install git`
-   `sudo apt-get install arm-none-eabi`
-  `sudo apt-get install libnewlib-arm-none-eabi`
-   `sudo apt-get install bossa-cli`
-   `sudo apt-get install doxygen`
-   `sudo apt-get install build-essential`

After installing all the packages clone the R2D2-Build(add link to this) repo by typing the following command `git clone add link here`
At this point everything should be working fine.

## Working with it:

**Note:** On linux platforms the `bmptk-make` command should be replaced with `make`

### Creating a new module:
1. Go to the `modules` directory located in `<path-to-your-repo>/R2D2-build/modules`
2. Start work on a module by using `git clone https://github.com/R2D2-2019/<name-of-module>` If there are already files in the repo you can skip step 3 if it is the first time go to step 3.
3. If it is the first time you start work on the module the repo will be empty. Copy the contents of the template-arduino or template-native directory to the folder of your module. **note:** native projects are for running on your pc and arduino for flashing to the due.

### Writing code:

 1. In the folder of your module that you just created there is a folder named "code" go to this directory.
 2. The "main.cpp" will remain in this directory
 3. .hpp files need to be located in the "headers" folder
 4. .cpp files need to be located in the "src" folder.

**Note:** Include your header files in the following fashion `<some_header.hpp>` the Makefile will find it for you, so you don't need to add the full path to your source files.
**Warning:** <u>ALL</u> .hpp files located in the "header" directory and <u>All</u> .cpp files located in the "src" directory will be automatically added to your compile sequence. To prevent errors don't place unfinished or non-compiling files in these folders. An other solution can be to rename them to .txt.

### Building a project:

 1. Open a command prompt 
 2. Use cd command to go to the directory `<path-to-your-repo>/R2D2-build/modules/<name-of-your-module>/code`
 3. In this directory run the following command `bmptk-make build`

 ### Flashing or running a project:

 1. Open a command prompt 
 2. Use cd command to go to the directory `<path-to-your-repo>/R2D2-build/modules/<name-of-your-module>/code`
 3. In this directory run the following command `bmptk-make run`
 4. The code will flash to your arduino or the .exe will start when a native project
**Note:** On linux make sure you have comport acces or run `sudo make run`
**Note:** Default COM port on windows is COM2, On linux the default is ttyUSB0.

### Cleaning a project:
 1. Open a command prompt 
 2. Use cd command to go to the directory `<path-to-your-repo>/R2D2-build/modules/<name-of-your-module>/code`
 3. In this directory run the following command `bmptk-make clean`

### Changing a project from arduino to native:
If you started a project for the arduino or for native and you realize you want to change you can do so by opening the "Makefile" located in < your-module-name>/code. The last line of this file contains the following `include $(RELATIVE)Makefile.native` change the .native to .due respectively.

### Writing tests:
In the module folder there is a folder named "test" this folder only contains a "main.cpp" and a "Makefile". This main wil contain Catch2 test cases. More explanation and requirements will be posted on this page later on. For now only work in the "code" directory.

### IDE/Editor:
Basically every editor is allowed as long as your code is compliant with the [code style guide](https://github.com/R2D2-2019/R2D2-2019/wiki/Style-Guide). Although we recommend using  [VS code](https://github.com/R2D2-2019/R2D2-2019/wiki/VS-Code) or [CLion](https://github.com/R2D2-2019/R2D2-2019/wiki/CLion) see the dedicated pages for both of these editors on a guide how to install those. So you are free to chose your own editor but if you experience problems we are not obligated to help you with it.



> Written with [StackEdit](https://stackedit.io/).
