# PX4_UDP_Connection module for AMLS system


![Logo](https://github.com/XxOinvizioNxX/Liberty-Way/blob/main/git_images/logo_book.png "Logo")

This project was created as a part of project activities subject of Moscow Polytech University by the 181-311 group.
It also participates in the CopterHack 2021 hackathon from Copter Express.

-----------

- **Liberty-Way Project:** https://github.com/XxOinvizioNxX/Liberty-Way
- **Eitude Project:** https://github.com/XxOinvizioNxX/Eitude

-----------

This project was made to simulate a drone's movements with prediction of its next GPS location. 

It has a track with points that have parameters of latitude, longitude and altitude. 
After the first waypoint is set, it is required for the user to enter only meters for the drone to go. Then the code calculates the next waypoint and adds the it to the track which is, after every way point is set, being passed by the drone model. 
