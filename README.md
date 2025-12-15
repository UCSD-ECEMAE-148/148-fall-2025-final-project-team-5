<h1>148-fall-2025-final-project-team-5</h1>
<p>148-fall-2025-final-project-team-5 created by GitHub Classroom</p>


<!-- PROJECT LOGO -->
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="images/UCSDLogo_JSOE_BlueGold.png" alt="UCSD Jacobs School of Engineering Logo" width="400" height="100" />
  </a>
  <h3>ECE/MAE148 Final Project</h3>
  <p>
    Team 5 • RoboDog • Fall 2025
  </p>
</div>

<hr />

<h2 id="table-of-contents"><b>Table of Contents</b></h2>
<ul>
  <li><a href="#team-members">Team Members</a></li>
  <li><a href="#final-project">Final Project</a></li>
  <li><a href="#original-goals">Original Goals</a></li>
  <li><a href="#goals-we-met">Goals We Met</a></li>
  <li><a href="#our-hopes-and-dreams">Our Hopes and Dreams</a></li>
  <li><a href="#stretch-goals">If We Had More Time</a></li>
  <li><a href="#final-project-videos">Final Project Videos</a></li>
  <li><a href="#hardware">Hardware</a></li>
  <li><a href="#software-design">Software Design</a></li>
  <li><a href="#ros2-network">ROS2 Network</a></li>
  <li><a href="#gantt-chart">Gantt Chart</a></li>
  <li><a href="#course-deliverables">Course Deliverables</a></li>
  <li><a href="#acknowledgments">Acknowledgments</a></li>
  <li><a href="#contact">Contact</a></li>
</ul>

<hr />

<h2 id="team-members">Team Members</h2>
<ul>
  <li>Luis David García Ríos – Electrical &amp; Computer Engineering</li>
  <li>Nemo Quan Dinh – Mechanical &amp; Aerospace Engineering</li>
  <li>Danny Salter – Mechanical &amp; Aerospace Engineering</li>
  <li>Anthony Flores-Mendez – Mechanical &amp; Aerospace Engineering</li>
</ul>

<h2 id="final-project">Final Project</h2>
<p>
  <strong>RoboDog</strong> is an autonomous police-dog–style robot that follows hand gestures, performs
  room-clearing maneuvers, scans for people, identifies who is holding a toy gun, and exits safely
  while reporting its findings.
</p>
<p>
  The robot uses:
</p>
<ul>
  <li><strong>Mediapipe</strong> landmark-based hand gesture detection</li>
  <li><strong>ROS2</strong> distributed control</li>
  <li><strong>RoboflowOakD</strong> object detection</li>
  <li><strong>Speaker output</strong> for audible reporting</li>
</ul>

<hr />

<h2 id="original-goals">Original Goals</h2>

<h3>Must Have</h3>
<ul>
  <li>Detect and follow officer hand gestures</li>
  <li>Sweep/clear a room or hallway based on an initial gesture</li>
  <li>Detect people and classify armed vs. unarmed</li>
  <li>Safely exit the room</li>
</ul>

<h3>Nice to Have</h3>
<ul>
  <li>Announce number of armed persons after exiting</li>
  <li>Add LIDAR + 1-axis gimbal for enhanced room scanning</li>
</ul>

<hr />

<h2 id="goals-we-met">Goals We Met</h2>

<h3>1. Hand Gesture Control</h3>
<p>
  Used Mediapipe hand landmarks to detect gestures. The robot accepts gesture commands such as
  <em>Forward, Reverse, HALT, Turn Left, Turn Right, Straight</em>, plus <em>room clear</em> maneuvers.
</p>

<h3>2. Room Sweep Autonomous Behavior</h3>
<p>
  The robot performs:
</p>
<ul>
  <li>360° room clear</li>
  <li>Left hall clear</li>
  <li>Right hall clear</li>
</ul>

<h3>3. Object Detection</h3>
<p>
  Using RoboflowOakD, the system detects:
</p>
<ul>
  <li><strong>Person</strong></li>
  <li><strong>Gun</strong></li>
  <li><strong>Person holding gun</strong></li>
</ul>
<p>
  A speaker node announces:
</p>
<ul>
  <li>People detected</li>
  <li>Armed individuals</li>
  <li>Room cleared</li>
</ul>

<h3>4. ROS2 System Architecture</h3>
<p>
  Modular node design for gesture, detection, navigation, speaker, and VESC control, as shown
  in the network diagram.
</p>

<hr />

<h2 id="our-hopes-and-dreams">Our Hopes and Dreams</h2>
<p>
  Our intended advanced version included:
</p>
<ul>
  <li>Reliable LIDAR + camera fusion</li>
  <li>Smoother throttle control via hand gestures</li>
  <li>A more robust gesture vocabulary</li>
  <li>Comprehensive 360° room scanning using gimbal rotation instead of driving loops</li>
</ul>

<hr />

<h2 id="stretch-goals">If We Had More Time</h2>
<ul>
  <li>2-axis gimbal for full panoramic scans without moving the car</li>
  <li>Improved LIDAR integration to validate detections and assist in low light</li>
  <li>Tighter gesture-to-command integration between object detection and command node</li>
</ul>

<hr />

<h2 id="final-project-videos">Final Project Videos</h2>
<p>
  See below for the final product videos.
</p>

<div align="center">
  <p><strong>Hand Gesture Demo</strong></p>
  <a href="videos/robocar_hand_detection_demo.mp4">
    <img src="images/ezgif-3500ed4f191ddbb8.gif" width="600" />
  </a>

  <p><strong>Room Sweep + Gun Detection</strong></p>
  <a href="videos/room%20clearing%20control%20demo%20(1).mp4">
    <img src="images/ezgif-371a4f158e13b9f1.gif" width="600" />
  </a>

</div>

<hr />

<h2 id="hardware">Hardware</h2>
<ul>
  <li>Raspberry Pi </li>
  <li>OAK-D and USB webcam</li>
  <li>VESC motor controller</li>
  <li>Speaker</li>
</ul>

<div align="center">
  <p><strong>RoboDog Platform</strong></p>
  <img src="images/robodog_final_assembly.jpg" alt="RoboDog Final Assembly" width="600" />

</div>

<hr />

<h2 id="software-design">Software Design</h2>

<h3>Hand Gesture Detection</h3>
<p>
  Uses Mediapipe hand landmarks to extract x, y, z positions of fingers and classify gestures such as:
</p>
<ul>
  <li>Forward</li>
  <li>Reverse</li>
  <li>HALT</li>
  <li>Turn Left</li>
  <li>Turn Right</li>
  <li>360 Room Clear</li>
  <li>Left</li>
</ul>
  
<hr />

<h2 id="ros2-network">ROS 2 Network</h2>
<p align="center">
  <img src="images/ros_architecture.png" alt="ROS Architecture" width="600">
</p>

<hr />

<h2 id="#gantt-chart">Gantt Chart</h2>
<p align="center">
  <img src="images/gantt.png" alt="Gantt Chart" width="600">
</p>

<hr />

<h2 id="#course-deliverables">Course Deliverables</h2>
<p>
Insert videos here
</p>

<hr />

<h2 id="#acknowledgments">Acknowledgements</h2>
<p>
  We would like to acknowledge and express our gratitude to the staff behind this class, Professor Silberman, and the TAs Winston, and Aryan (the GOAT) for all the assistance with troubleshooting and programming issues. Thank you for the quarter and putting up with us!
</p>

<h2 id="">Contact</h2>
<p>
  <div style="border: 1px solid #ccc; padding: 15px; border-radius: 8px; max-width: 400px;">

<h3>Contact Information</h3>

<ul>
  <li><strong>Luis</strong>: ldg002@ucsd.edu</li>
  <li><strong>Nemo</strong>: nqdinh@ucsd.edu</li>
  <li><strong>Danny</strong>: desalter@ucsd.edu</li>
  <li><strong>Anthony</strong>: afloresmendez@ucsd.edu</li>
</ul>

</div>
</p>
