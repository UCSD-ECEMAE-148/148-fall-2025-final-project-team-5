# 148-fall-2025-final-project-team-5
148-fall-2025-final-project-148-fall-2025-final-project-team-5 created by GitHub Classroom

<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>RoboDog - ECE/MAE148 Final Project Team 5</title>
  <style>
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      line-height: 1.6;
      max-width: 900px;
      margin: 0 auto;
      padding: 2rem 1rem 4rem;
    }
    h1, h2, h3 {
      font-weight: 700;
    }
    h1 {
      font-size: 2.2rem;
      margin-bottom: 0.2rem;
    }
    h2 {
      margin-top: 2rem;
      border-bottom: 2px solid #e5e5e5;
      padding-bottom: 0.3rem;
    }
    h3 {
      margin-top: 1.4rem;
    }
    a {
      color: #0070c9;
      text-decoration: none;
    }
    a:hover {
      text-decoration: underline;
    }
    .center {
      text-align: center;
    }
    .logo-img {
      max-width: 400px;
      height: auto;
    }
    .hero-subtitle {
      font-size: 1.1rem;
      margin-top: 0.4rem;
    }
    ul {
      margin-left: 1.4rem;
    }
    code {
      background-color: #f6f8fa;
      padding: 2px 4px;
      border-radius: 4px;
      font-size: 0.95em;
    }
    hr {
      margin: 2rem 0;
      border: none;
      border-top: 1px solid #ddd;
    }
    .img-large {
      max-width: 100%;
      height: auto;
      display: block;
      margin: 0.5rem auto;
    }
    .toc ul {
      list-style: none;
      padding-left: 0;
    }
    .toc li {
      margin: 0.15rem 0;
    }
    .contact-list, .team-list {
      list-style: none;
      padding-left: 0;
    }
  </style>
</head>
<body>

  <h1>148-fall-2025-final-project-team-5</h1>
  <p>148-fall-2025-final-project-team-5 created by GitHub Classroom</p>

  <!-- PROJECT LOGO -->
  <div class="center">
    <a href="https://jacobsschool.ucsd.edu/">
      <img src="images/UCSDLogo_JSOE_BlueGold.png" alt="UCSD Jacobs School of Engineering Logo" class="logo-img" />
    </a>
    <h3>ECE/MAE148 Final Project</h3>
    <p class="hero-subtitle">
      Team 5 • RoboDog • Fall 2025
    </p>
  </div>

  <hr />

  <h2 id="table-of-contents"><b>Table of Contents</b></h2>
  <div class="toc">
    <ul>
      <li><a href="#team-members">Team Members</a></li>
      <li><a href="#final-project">Final Project</a></li>
      <li><a href="#original-goals">Original Goals</a></li>
      <li><a href="#goals-we-met">Goals We Met</a></li>
      <li><a href="#our-hopes-and-dreams">Our Hopes and Dreams</a></li>
      <li><a href="#stretch-goals">Stretch Goals</a></li>
      <li><a href="#final-project-videos">Final Project Videos</a></li>
      <li><a href="#hardware">Hardware</a></li>
      <li><a href="#software-design">Software Design</a></li>
      <li><a href="#ros2-network">ROS2 Network</a></li>
      <li><a href="#gantt-chart">Gantt Chart</a></li>
      <li><a href="#course-deliverables">Course Deliverables</a></li>
      <li><a href="#acknowledgments">Acknowledgments</a></li>
      <li><a href="#contact">Contact</a></li>
    </ul>
  </div>

  <hr />

  <h2 id="team-members">Team Members</h2>
  <ul class="team-list">
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
    <li><strong>Roboflow / YOLO</strong> object detection</li>
    <li><strong>Speaker output</strong> for audible reporting</li>
    <li>Optional <strong>LIDAR + gimbal</strong> for extended scanning</li>
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
    Using Roboflow/YOLO, the system detects:
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

  <h2 id="stretch-goals">Stretch Goals</h2>
  <ul>
    <li>2-axis gimbal for full panoramic scans without moving the car</li>
    <li>Improved LIDAR integration to validate detections and assist in low light</li>
    <li>Better hardware acceleration using a Pi HAT or upgraded compute module</li>
    <li>Tighter gesture-to-command integration between object detection and command node</li>
  </ul>

  <hr />

  <h2 id="final-project-videos">Final Project Videos</h2>
  <p>
    Replace the image and video file names with your actual files in <code>images/</code>.
  </p>

  <div class="center">
    <p><strong>Hand Gesture Demo</strong></p>
    <a href="videos/hand_control_demo.mp4">
      <img src="images/hand_control_demo.gif" alt="Hand Control Demo" class="img-large" />
    </a>

    <p><strong>Room Sweep + Gun Detection</strong></p>
    <a href="videos/room_sweep_demo.mp4">
      <img src="images/room_sweep_demo.gif" alt="Room Sweep Gun Detection Demo" class="img-large" />
    </a>
  </div>

  <hr />

  <h2 id="hardware">Hardware</h2>
  <ul>
    <li>Raspberry Pi / Jetson (compute platform)</li>
    <li>OAK-D or USB webcam</li>
    <li>VESC motor controller</li>
    <li>Speaker</li>
    <li>Optional LIDAR + gimbal</li>
  </ul>

  <div class="center">
    <p><strong>RoboDog Platform</strong></p>
    <img src="images/robodog_final_assembly.jpg" alt="RoboDog Final Assembly" class="img-large" />

    <p><strong>Wiring / Electronics Layout</strong></p>
    <img src="images/wiring_diagram.png" alt="Wiring Diagram" class="img-large" />
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
    <li>Left/Right Hall Clear</li>
  </ul>
  <p>
    Commands are published to a <code>/gesture</code> topic and executed by a <code>cmd_node</code>.
  </p>

  <h3>Object Detection Node</h3>
  <p>
    Detects persons, guns, and persons holding guns, then publishes detection messages to topics such as:
  </p>
  <ul>
    <li><code>/people_count</code></li>
    <li><code>/armed_count</code></li>
    <li><code>/speaker</code></li>
  </ul>

  <h3>Speaker Node</h3>
  <p>
    Listens to detection topics and announces:
  </p>
  <ul>
    <li>Total people detected</li>
    <li>Total armed assailants</li>
    <li>Room cleared status</li>
  </ul>

  <hr />

  <h2 id="ros2-network">ROS2 Network</h2>
  <p>
    The ROS2 graph includes nodes such as:
  </p>
  <ul>
    <li><code>gesture_detection_node.py</code></li>
    <li><code>cmd_node.py</code></li>
    <li><code>gun_detection_node.py</code></li>
    <li><code>speaker_node.py</code></li>
    <li>VESC control node</li>
  </ul>
  <p>
    These nodes communicate via ROS2 topics to create a modular, fault-tolerant control architecture.
  </p>

  <div class="center">
    <p><strong>ROS2 Network Diagram</strong></p>
    <img src="images/ros2_network_diagram.png" alt="ROS2 Network Diagram" class="img-large" />
  </div>

  <hr />

  <h2 id="gantt-chart">Gantt Chart</h2>
  <p>
    Our project schedule and major milestones are summarized in the Gantt chart below.
  </p>
  <img src="images/Gantt_Chart.png" alt="Gantt Chart" class="img-large" />

  <hr />

  <h2 id="course-deliverables">Course Deliverables</h2>
  <p>
    Link to slides, reports, and other required deliverables:
  </p>
  <p>
    <a href="https://drive.google.com/drive/folders/19BW6AHKIXl78ZI0pPDGdtria1HKv7pat?usp=drive_link" target="_blank" rel="noopener noreferrer">
      <img src="images/course_deliverables.png" alt="Course Deliverables" class="img-large" />
    </a>
  </p>

  <hr />

  <h2 id="acknowledgments">Acknowledgments</h2>
  <p>
    Special thanks to:
  </p>
  <ul>
    <li>Professor Jack Silberman</li>
    <li>Winston</li>
    <li>Alexander</li>
  </ul>
  <p>
    For their guidance and support during Fall 2025 ECE/MAE148.
  </p>

  <hr />

  <h2 id="contact">Contact</h2>
  <ul class="contact-list">
    <li>Luis | <a href="mailto:luis@example.com">luis@example.com</a></li>
    <li>Nemo | <a href="mailto:nemo@example.com">nemo@example.com</a></li>
    <li>Danny | <a href="mailto:danny@example.com">danny@example.com</a></li>
    <li>Anthony | <a href="mailto:anthony@example.com">anthony@example.com</a></li>
  </ul>

</body>
</html>
