********************************************************************************Camellia Dragons Code Release 2017
********************************************************************************
Camellia Dragons Code Release 2017 was basically developed based on B-Human Code Release 2016 (https://github.com/bhuman/BHumanCodeRelease).

Three major modules of Priority, CascadePerceptor and PkGoalieBallPerceptor were developped under B-Human Code Release 2015 (https://github.com/bhuman/BHumanCodeRelease).

When you build the system you might receive some warnings. 
But it will work without any problems. 

### Introduction
The system requires OpenCV library (https://github.com/opencv/opencv). 
We confirmed with OpenCV 3.1.
After installing OpenCV library, you should follow the document of B-Human Code Release 2016 (https://github.com/bhuman/BHumanCodeRelease). 

### Switching modules
You should switch two modes between the team competition and the penalty shootout. 
Please modify a valiable 'location' in Config/settings.cfg as follows. 
- Default: team competition
- PkGoalie: goalie at penalty shootout
- PkStriker: striker at penalty shootout

### Adding and modifying OpenCV library
OpenCV 3.1 is supporsed to be used. 
If installing different versions of OpenCV you should modify the libraries and headers in Util/opencv. 
You should specify a path to OpenCV in Make/Common/Nao.mare and Make/Common/SimulatedNao.mare. 
If you want to change the cascade please modify Config/Cascade/cascade.xml. 
********************************************************************************
