# Easy-SCANeR: Developing a Next-Generation Driving Simulator

You can find compiled documentation in `docs-compiled` folder.
The pyscaner package already compiled with instuctions is present in `pyscaner-compiled.zip`.


## Description
The software is meant to be used with the SCANeR software developed by AVSimulation. It allows you to have the autonomous vehicles in SCANeR adhere to more realistic and complex traffic models rather than the default behaviour that AVSimulation has implemented.
Our client deemed this necessary, as the default implementation for the autonomous vehicles is simplistic and unrealistic. This application should fix that problem when used correctly, which allows our client to perform more realistic experiments.

## Update documentation
1. In your console, navigate to `sphinx-docs/docs/`
2. Type `doxygen`. If everything went well, it should output `Generating XML output for file [filename]` and end with `finished...`.
3. Type `./make clean | ./make html`. If everything went well, the output should end with `The HTML pages are in build\html.`.

Depending on your machine, some of the commands need to be slightly altered (e.g., `./make` could be `make`).

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
The application is written in C++ and should be built using CMake. The CMake setup is already setup for you.

To install the software, you will need to download the source code and open it in an IDE of your choice. We have worked with Visual Studio.
Then, use the IDE to build all executables. They will be located in `out/build/x64-Debug/`.

## Authors and acknowledgment
All code was developed by Ties Bloemen, Ois�n Hageman, Brent Meeusen, Medard Szilv�sy and Pavel Verigo.
