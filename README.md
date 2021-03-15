# PX4_UDP_Connection module for AMLS system


![Logo](https://github.com/XxOinvizioNxX/Liberty-Way/blob/main/git_images/logo_book.png "Logo")

This project was created as a part of project activities subject of Moscow Polytech University by the 181-311 group.
It also participates in the CopterHack 2021 hackathon from Copter Express.

-----------

## AMLS Projects:

- **Liberty-Way Project:** https://github.com/XxOinvizioNxX/Liberty-Way
- **Eitude Project:** https://github.com/XxOinvizioNxX/Eitude

-----------

This project was made to simulate a drone's movements with prediction of its next GPS location. 

The tools that were used for that purpose: PX4 firmware and Gazebo simulator. 

The script connects with gazebo via default UDP port through local network and can be refactored to establish connection between Virtualbox machine and the Host machine with using port forwarding and IP adjustment - the script can connect to VM. 

The application consists of a track with points that have parameters of _latitude_, _longitude_ and _altitude_.

After the first waypoint is set, it is required for the user to enter meters for the drone to go. Then the code calculates the next waypoint and adds it to the track which is, after every of the waypoints are set, being passed by the drone model. 

When the tracking is started, the application outputs current GPS location, estimated next GPS location and the error between the next true GPS location and the estimated one. 
