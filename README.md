# Autonomous Robots: Kalman Filter
Assignments for my Udemy course on Kalman Filters.

Course link: www.udemy.com/course/autonomous-robots-kalman-filter/

Required Packages <br>
python=3.7.4 <br>
numpy=1.16.4 <br>
matplotlib=3.1.0 <br>

<h4>Assignment 1: Toy Implementation</h4>
-> Intro to localization and principle of Kalman filter using simple model of car in 1D spacer<br>
-> Used to explain prediction of speed based on collected data and new measurement. i.e. The speed cannot change abruptly

<h4>Assignment 2: 1D Kalman Filter</h4>
-> Same problem as in assignment 1, but this time Linear Kalman filter is used to localize car<br>
-> Explains how to setup Kalman filter and what individual matrices are used for

<h4>Assignment 2: 2D Kalman Filter</h4>
-> Goal is to localize car as precise as possible while driving on a street with turns and traffic lights <br>
-> Program will not only recieve measurement but also vector U (user input)

<h4>Assignment 3: Traffic light prediction</h4>
-> In this assignment car is approaching intersection and at some point traffic light will change to red <br>
-> Car need to decide if it will make to other side. And based on this prediction needs to decide to stop or continue<br>
-> In first scenario car makes prediction based on current state <br>
-> In second scenario car makes prediction based on current state but also on fact that it is allowed to raise speed for 1 sec


<h3> Kalman Filter </h3>

<h4>x = State Vector </h4>
<h4>P = Uncertainty Matrix </h4>
<h4>F = State Transition Matrix </h4>
<h4>H = Measurement Matrix </h4>
<h4>R = Measurement Uncertainty </h4>
