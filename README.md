# Autonomous Robots: Kalman Filter
Assignments for my Udemy course on Kalman Filters.

Course link: www.udemy.com/course/autonomous-robots-kalman-filter/

Required Packages <br>
python=3.7.4 <br>
numpy=1.16.4 <br>
matplotlib=3.1.0 <br>

<h3> Kalman Filter </h3>
<p>Detailed infomation <a href ="https://en.wikipedia.org/wiki/Kalman_filter"> here</a> !</p>
<p> - represent the estimated state as a probability distribution</p>
<p> - Represent the measurements or obervations of the current state (or function of the state) as a probability distribution</p>
<p> - Fuse the two distributions to get a better estimate (Bayes Theorem) </p>
<p> - Prediction process increase the uncertainity with time </p>
<p> - Update/Measurement process decrease the uncertainity</p>
<p> - Kalman Filter allows use to do this numerically and mathematically simply, by making a few assumptions about the probabilty distributions and a few other properties of the dynamic system</p>
<p> First we design 5 matrix below and then we used them in 2 step:</p>
<p> 1. Predict new state and state uncertainity </p>
<p> 2. Measure and update state and state uncertainity</p>

<h4>x = State Vector </h4>
<p>-> Contains all the states we are interested in or that are need to solve given problem </p>
<p>-> State can be position, speed, acceleration etc. </p>
<p>-> Size of the matrix is n x 1, where n is number of states </p>
<p>-> At the beginning we populate this matrix with some initial conditions </p>

<h4>P = Uncertainty Matrix </h4>
<p>-> Tells us how much uncertain we are about individual states </p>
<p>-> Uncertainty corresponding to states are being filled on to diagonal </p>
<p>-> Uncertainty 0 means that we are 100% sure about given state </p>
<p>-> Size of the matrix is n x n, where n is number of states </p>
<p>-> At the begging we populate this matric with information about how much uncertainty we are about given states </p>

<h4>F = State Transition Matrix </h4>
<p>-> Tells us how we can get from current state to next state i.e how to get from x<sub>t</sub> to x<sub>t+1</sub></p>
<p>-> Size of the matrix is n x n, where n is number of states </p> 

<h4>H = Measurement Matrix </h4>
<p>-> Tells us which measurement correspond to which state </p>
<p>-> Usually consist only 0 or 1, meaning measurement correspond to state = 1 or do not = 0 </p>
<p>-> Size of the matrix is m x n, where m is number of measurements and n is number of states </p>

<h4>R = Measurement Uncertainty </h4>
<p>-> Tells us how much uncertain we are about individual measurement </p>
<p>-> Uncertainty corresponding to measurement are being filled on to diagonal </p>
<p>-> Uncertainty 0 means that we are 100% sure about given measurement </p>
<p>-> Size of the matrix is m x m, where m is number of measurements</p>
<p>-> At the begging we populate this matric with information about how much uncertainty we are about measurements</p>

<h4>Predict</h4>
  <p>x = Fx </p>
  <p>P = FPF<sup>T</sup></p>
  
<h4>Measure and Update</h4>
  <p>Z = Measurements</p>
  <p>y = Z<sup>T</sup> - Hx</p>
  <p>S = HPH<sup>T</sup> + R</p>
  <p>K = PH<sup>T</sup>S<sup>-1</sup></p>
  <p>x = x + Ky</p>
  <p>P = (I - KH)P      (I = Identity matric)</p>

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
  
