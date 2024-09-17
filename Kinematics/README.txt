###############################################################################
#                        MEDLAB EPOS4 LINUX LIBRARY                           #
###############################################################################
#  Language:  C++                                                             #
#                                                                             #
#  Jason Shrand, Vanderbilt University 2022. All rights reserved.             #
#  DO NOT REDISTRIBUTE WITHOUT PERMISSION                                     #
#                                                                             #
#  This software is distributed WITHOUT ANY WARRANTY; without even            #
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR        #
#  PURPOSE. See the above notices for more information.                       #
###############################################################################




                                1) REQUIREMENTS                               
###############################################################################
TODO




                                2) INSTALLATION                               
###############################################################################

+-------------------------------------+
| 2.3) Build the MEDLab CTRKinematics  |
+-------------------------------------+

    1) Open a terminal in the CTRKinematics folder, and run the following command:
    
    ./build.sh
    
    
+-------------------------------------+
| 2.4) System Installation            |
+-------------------------------------+

    1) Open a terminal in the CTRKinematics/build folder, and run the following command:

    sudo make install
    
    
                                    3) USE IN CMAKELIST                               
###############################################################################
To include and use this library in another file, add the following to your CMakeList.
```cmake
include_directories(
    include
    "/usr/local/include/CTRKinematics"
)

# add link to the topmost target (i.e. whichever file gets added to all the others and uses the library)
target_link_libraries(TARGET "/usr/local/lib/CTRKinematics/libCTRKinematics.so")

```

