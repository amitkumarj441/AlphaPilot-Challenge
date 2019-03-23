# Test 3 - AlphaPilot Qualifier (Guidance, Navigation and Control)

The third component of AlphaPilot qualifications will focus on a team’s ability to design algorithms for the guidance, navigation, and control (GNC) of an autonomous drone. The test will utilize a simulator framework that provides users with the tools needed to test their drone racing algorithms using realistic dynamics and exteroceptive sensors. These skills are essential for competition in AlphaPilot and the test is considered a precursor to work conducted by Finalist teams in preparation for each AIRR race event.

## Goal
Teams must develop GNC algorithms to fly a simulated drone through a structured test environment utilizing a typical ACRO/RATE flight mode for control inputs and exteroceptive sensors for feedback. Teams are tasked with:

 - Developing GNC algorithms to pilot an autonomous drone through FlightGoggles
 - Describing their GNC algorithms in a 2-page Technical Report

The GNC algorithms must be capable of navigating through gates (see Figure 1) in a FlightGoggles challenge course. The objective is to pass through all gates in the defined order, as quickly as possible, without crashing.

### Resources
Test #3 is built on the Massachusetts Institute of Technology (MIT) FlightGoggles simulator, a first-of-its-kind virtual reality environment for drone research and development. MIT has modified FlightGoggles to include software for a drone racing simulation environment based on the Unity3D game engine. It includes Robotic Operating System (ROS) binding for integrating autonomy software and Unity assets for environments that resemble drone racing scenarios.

The FlightGoggles simulator and several training courses are now available open-source to the public and are hosted on GitHub. You can find the source code for the simulator, more information about how to get started, and some example challenge files here: http://flightgoggles.mit.edu/

#### Vehicle Model

The simulator emulates a high-performance drone racer with high pitch rate and collective thrust command inputs. Note that this drone is not an exact model of the one used by AlphaPilot in AIRR race events. A Vehicle Dynamics node will keep track of the simulation clock and adjust the simulation clock ratio, which can be seamlessly lowered in real-time if the host computer is unable to provide real-time performance due to high-load or can be raised if faster real-time simulation is desired. More details about the drone (vehicle dynamics, sensor models, specifications, etc.) will be added to the GitHub repo.

#### Training Courses & Course Developer:

The simulator environment includes several pre-developed challenge courses and an open-map course creator for easy development and testing. Teams can use both resources for practice and may submit performance data on a Leaderboard Challenge Course to be included on a HeroX Leaderboard. The Final Challenge Course files will be developed by the AlphaPilot administration and kept on a separate instance. This course will be used for scoring submitted algorithms and determining teams final Test 3 Algorithm Scores.

The simulator is run with a Challenge File in YAML format which corresponds to a challenge course, and it will produce a Results YAML File after it is done. The Challenge File describes the following details:

   - Initial position and orientation of the drone
   - Challenge Name
   - Timeout Specification
   - A list of gate names
   - Width of all gates
   - Location of all the gates, determined by four points that make a rectangle 
   
   
#### Simulator Output:

The simulator is equipped with a Reporter node that tracks ground truth information and outputs, via command line, and yields metrics on algorithm performance as a Results YAML File for each run on a Challenge YAML File. The file will contain metrics on drone performance attributes useful in logging, evaluating, and visualizing algorithm performance.

Additionally, the Simulator Reporter will output a YAML file with details about the run including output one of the following results:

   - Interrupted: If the grader process is interrupted, e.g., via ctrl+c.
   - Timeout: The timeout on the challenge file is reached
   - Crashed: The drone crashed into an obstacle in the sim
   - Completed: The drone passed through the last gate in the sequence.

The Reporter will also keep track of when the drone reaches each gate and record the time. If a gate in order is skipped, then the grader will put down “Success: False” for that gate.

With the Leaderboard Challenge Course files, teams have scripts to help with scoring the Results YAML file and outputting a Scores YAML file. See the included ‘Scorer Scripts’ in the challenge files as well as the README_v2.md file for more details on running these scripts. Teams may submit their Scores YAML file of their summarized results for scoring and placement on the Leaderboard.

The Final Challenge Course files will not be released to teams and will be used to evaluate each team’s algorithm and determine their final FlightGoggles score. See the ‘Testing’ section below for more details on Leaderboard and Final Testing.

   
#### Algorithm Requirements:

Teams must develop GNC algorithms to fly the simulated drone through a structured test environment utilizing a typical ACRO/RATE flight mode for control inputs and only on-board sensory feedback. As is typical in drone racing, teams know the approximate locations of their starting point and the gate locations, and they must deal with slight variations during the actual race. In Test 3, teams are challenged to create a single algorithm that can be tested on 25 slight variations of the same Challenge Course.

Teams are given Challenge YAML files on which teams will be tested that indicate the location of the initial conditions for each exam. For Leaderboard Testing, teams are given all 25 Challenge YAML Files and self-report their scores. For Final Testing, teams are not given the variations on the Challenge YAML Files, and their algorithms will be run and scored by HeroX.

Algorithms are not allowed to use ground-truth state data; only inputs from their sensors. These vehicle and sensor feeds are modelled to include real-world errors like thrust-limited motors and rate-based control drift. Teams will have access to this noisy data, as well as some visual processing outputs. In summary, the allowable sensory inputs for algorithms include:

   - Photorealistic 60Hz RGB camera feeds – mono and stereo allowed
   - “Perfect” (not noisy) gate detection algorithm output (polygon coordinates) in the camera frame
   - “Perfect” (not noisy) IR marker locations in the camera frame
   - Noisy IMU data
   - Noisy downward-facing laser range-finder to aid altitude estimation. The laser range finder points in the negative z direction. Its measurements are defined in the drone body frame, and thus range measurements will have a negative value.

Using these sensory inputs, teams are permitted to conduct any processing that they desire. No visual-based obstacle avoidance is necessary (course paths are mostly clear of obstacles). That being said, it is possible to crash into things and fail the run.

    Note: AlphaPilot is aware that it will be possible to back-out the exact global 3D locations of gates. Teams are not allowed to utilize this data in their algorithms, and any teams that do so will be flagged during judging.

Algorithms should be built upon the FlightGoggles ROS framework. Teams have the flexibility to utilize any programming language in ROS nodes. Teams may leverage open-source algorithms and data libraries to assist in their design. The written submissions should document important elements of your approach (e.g. language, architecture) and explain why you believe this approach is both favorable for performance in Test #3 and scalable to the larger AlphaPilot challenge.

Entries to both the Leaderboard and Final Exam must be submitted using the Test #3 entry form provided on the AlphaPilot HeroX website. Entries for the Leaderboard Challenge Course must be a single Scores YAML file and submitted as an attachment. This file contains scores from all 25 Results YAML files and is used by HeroX to calculate teams Leaderboard Algorithm Score (where top 5 scores are found and averaged).

Algorithm entries for the Final Challenge Course (see more details about ‘Final Testing’ below) must be submitted executable programs as well as source code before the final deadline. The final submission form will contain fields for attachment of a zip file of your executable code, source code, and technical report.

See the ‘Testing’ section for more details about how algorithms will be tested.

Note: GPU and CUDA drivers will already be installed in the testing environment. The following instance will be used at the basis for the testing environment: https://aws.amazon.com/marketplace/pp/B077GCZ4GR
