^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_footstep_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.9 (2016-03-23)
------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_footstep_planner] Add command_pose_stampped to footstep_marker
* [jsk_footstep_planner] Add bbox visalization and check goal status sanity before planning
* [jsk_footstep_planner] Remove points around the robot from robot_center_pointcloud before combine with robot_center_map_origin_points
* [jsk_footstep_planner] Support projection from marker
  Modified:
  - jsk_footstep_planner/include/jsk_footstep_planner/footstep_marker.h
  - jsk_footstep_planner/src/footstep_marker.cpp
* Merge pull request #562 from garaemon/collision-avoidance
  [jsk_footstep_planner] Support collision avoidance in footstep planner
* [jsk_footstep_planner] Support collision check
* [jsk_footstep_planner] Not use im_helpers::add3Dof2DControl for backward compatibility
* [jsk_footstep_marker] Move footstep_marker from jsk_interactive_marker
* [jsk_footstep_planner] Fix genjava problem with message_generation as build_depend
  Modified:
  - jsk_footstep_planner/package.xml
* [jsk_footstep_planner] Only consider forward step
  stride in heuristic estimation
  Modified:
  - jsk_footstep_planner/src/footstep_graph.cpp
* [jsk_footstep_planner] Do not check pointcloud if no perception mode
  Modified:
  - jsk_footstep_planner/src/footstep_planner.cpp
* [jsk_footstep_controller] Merge heightmap nodelets into multisense_laser nodelet to reduce tf related nodes
* [jsk_footstep_planner] Add successors definitions for jaxon, jaxon_red
  and hrp2jsknt
* Update .travis to jsk-travis 0.2.1 and enable ccache
* {jsk_calibration, jsk_footstep_planner}/README.md: fix section/subsection
* [jsk_footstep_planner] Measure perception duration in collaborative
  perception-planning scheduling
* [jsk_footstep_planner] Fix skip_cropping=true behavior
* [jsk_footstep_planner] Remove literal value from benchmark code
* [jsk_footstep_planner] Remove ROBOT env from sample launch files
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Iori Kumagai

0.1.8 (2015-11-02)
------------------

0.1.7 (2015-11-01)
------------------
* [jsk_footstep_planner] Add &allow-other-keys to
  fullbody-inverse-kinematics-with-standcoords.
  You can add :collision-check-robot-link-list and so on
* add variables to modify bounding box height
* [jsk_footstep_planner:footplace_sample]add sample file for footplace_manip
* [jsk_footstep_planner:footplace..]merge origin/master
* [jsk_footstep_planner:footplace~] debug output like normak ik
* fix minor bug in fullbody-inverse-kinematics-with-standcoords
* add args for ik with standcoords
* [jsk_footstep_planner] Ignore Z distance in heuristic computation
* Merge pull request `#488 <https://github.com/jsk-ros-pkg/jsk_control/issues/488>`_ from garaemon/remove-global-variable
  [jsk_footstep_planner] Remove global variable from footplace planning
* [jsk_footstep_planner] Fix indent of footplace_planner_for_manipulation.l
* [jsk_footstep_planner] Remove global variable from footplace planning
* change name of inverse-reachablity code
* add foot placement coords with ik
* [jsk_footstep_planner] Add script to convert
  jsk_footstep_msgs/FootstepArray to jsk_recognition_msgs/BoundingBox
* [jsk_footstep_controller] Update footstep planner parameter for hrp2
* [jsk_footstep_planner] Verify global location of footstep in projecting
  start and goal footstep
* [jsk_footstep_planner] Add global transition limit to verify global
  location of footstep
* [jsk_footstep_planner/simple_neighbored_graph.h] add missing include string
* change static polygon param
* [jsk_footstep_planner] Do not use jsk_pcl_ros, use jsk_recognition_utils
  instead of it.
  These commits are forgotten in previous commit.
* [jsk_footstep_planner] Use jsk_recognition_utils instead of jsk_pcl_ros
* [jsk_footstep_planner] Update stair model to more difficult one
* [jsk_pcl_ros] Fix handling of --enable_lazy_perception and
  --enable_local_movement options and printout graph info
* [jsk_footstep_planner] Add infoString method to print footstep graph property
* [jsk_pcl_ros] Do not raise exception when cvs has lack data
* [jsk_footstep_planner] Add simple launch file to preview models for benchmarking
* [jsk_footstep_planner] Add --only-save-image option to plotting script
* [jsk_footstep_planner] Add --verbose option to bench_footstep_planner.cpp
* [jsk_footstep_planner] Save to eps figure when visualizing benchmark plot
* [jsk_footstep_planner] build pointcloud model in more wider area
* [jsk_footstep_planner] Check ANNGridCell is already allocated
* [jsk_footstep_planner/bench_footstep_planner] Project start and goal
  footstep before taking benchmark
* [jsk_footstep_planner] Add anonymous flag to ros::init in benchmark program
* [jsk_footstep_planner] Add several args to disable perception and
  run planner with hrpsys/gazebo
* [jsk_footstep_planner] Add start-abc button for planner gui using with simulator
* [jsk_footstep_planner] Update benchmark program to specify a lot of parameters
* [jsk_footstep_controller, jsk_teleop_joy] Use footstep-controller.l and lock/unlock furutaractive
  model during exeucuting footsteps
* [jsk_footstep_planner] Fix indent
* [jsk_footstep_planner] Fix typo: crpping -> cropping
* [jsk_footstep_planner, controller] Add rviz GUI set for playing with footstep planner
* [jsk_footstep_planner] Use odom_init frame to publish plane for unseen region
* [jsk_footstep_controller/footcoords] Add odom_init frame which holds the pose when robot is put on the ground
* [jsk_footstep_planner] Add gaussian pointcloud to pointcloud generator
* Merge pull request `#414 <https://github.com/jsk-ros-pkg/jsk_control/issues/414>`_ from garaemon/default-body-on-odom
  [jsk_footstep_planner] Use body_on_odom frame as robot center frame
* [jsk_footstep_planner] Use body_on_odom frame as robot center frame
* [jsk_footstep_planner] Print error message about projection on rviz
* [jsk_footstep_controller] Add simple-footstep-controller as the most simplest footstep controller using
  :set-foot-steps
* [jsk_footstep_planner] Check pointcloud is available before projection
* [jsk_footstep_planner] Cleanup heightmap launch files
* Merge remote-tracking branch 'refs/remotes/origin/master' into crosscheck
* [jsk_footstep_planner] Implement cross check
* [jsk_footstep_planner] Add launch file to run footstep planner with heightmap
  integration
* [jsk_footstep_planner] Add text information on rviz
* [jsk_footstep_planner] Ignore warning message from pcl
* [jsk_footstep_planner] Fix projection around yaw axis orientation
* [jsk_footstep_planner] Add launch file for heightmap mapping
* Merge remote-tracking branch 'refs/remotes/origin/master' into hole-rate
  Conflicts:
  jsk_footstep_planner/src/pointcloud_model_generator.cpp
* [jsk_footstep_planner] Add ~hole_rate to simulate hole in pointcloud
* [jsk_footstep_planner] Publish pointcloud periodically from pointcloud_model_generator_node
* [jsk_footstep_planner] Just use kdtree nearest search in checking
  if footstep is on pointcloud
* [jsk_footstep_planner] add cost_weight and heuristic_weight parameter
* [jsk_footstep_planner] Update pointcloud to show close list and open
  list during planning
* [jsk_footstep_planner] Check value of transition when expanding nodes
* [jsk_footstep_planner] Use center of footprint to check if footprint is on pointcloud
* [jsk_footstep_planner] Project footprint with local search
* [jsk_footstep_planner] Add projection API to c++ footstep planner
* [jsk_footstep_planner] Add more parmeters to dynamic_reconfigure API of
  cpp footstep_planner
* [jsk_footstep_planner] Add perception sample with actionlib interface
* [jsk_footstep_planner] Add actionlib interface to C++ version of
  footstep planner. and add simplest smaple
* [jsk_footstep_planning] Visualize open and close list as pointcloud
* [jsk_footstep_planner] Fix ANNGrid search
* [jsk_footstep_planner] Skip planar region perception if footstep is
  already on pointcloud
* [jsk_footstep_planner] PointCloud approximate search based on 2-D grid
* [jsk_footstep_planner] Implement local movement if footstep is close to
  success of projection to pointcloud
* [jsk_footstep_planner] Check pointcloud model supports footprint
* [jsk_footstep_planner] Do not use SVD in perception
* [jsk_footstep_planner] Re-implement footstepHeuristicStepCost in
  computationally-efficient way.
  1. Do not use Eigen::Affine3f::rotation because it calls SVD internally.
  2. Do not cast to Eigen::AngleAxisf, just use cos(w/2) to compute angle
  from quaternion.
* [jsk_footstep_planner] Add profile function interface
* [jsk_footstep_planner] Add script to plot bench result
* [jsk_footstep_planning] Add program to bench footstep planning speed
* [jsk_footstep_planner] Add demonstration of footstep planning over curved and sloped surface
* [jsk_footstep_planner] Fix orientation of projected footstep
* [jsk_footstep_planner] Add timeout argument to solver
* [jsk_footstep_planner] Fix when footstep failed to project on planar region
* [jsk_footstep_planner] Planning with pointcloud model is implemented.
  We optimized perception phase by lazy-perception-in-planning technique:
  1) Do not detect planar region before planning
  2) Do not detect planar region until accurate pose of footstep is
  required
  3) use 2.5D pointcloud to get candidate pointcloud which footstep is placed on
* [jsk_footstep_planner] Add demo for curved surface
* [jsk_footstep_planner] Interactive demo of C++ footstep planner
* [jsk_footstep_planner] Use FootstepStateDiscreteCloseList for close list
* [jsk_footstep_planner] 2D footstep planning is implemented in C++
* [jsk_footstep_planner] Implement FootstepState and projection to pointcloud
* [jsk_footstep_planner] Add demo directory and install headers and library
* [jsk_footstep_planner] Implement C++ a* solver
* [jsk_footstep_planner] Initial commit of cpp graph library
* [jsk_footstep_planner] Update jaxon_red footprint region
* [jsk_footstep_planner, jsk_footstep_controller] Support HRP2JSKNT
* [jsk_footstep_planner, jsk_footstep_controller] Add USE_JOY option
* [jsk_footstep_planner, jsk_footstep_controller] Refactor launch file and
  add no_recog.launch
* [jsk_footstep_planner] Rename launch file to use ROBOT environment variable
* Contributors: Masaki Murooka, Ryohei Ueda, Yu Ohara, Yuki Furuta, Yusuke Oshiro

0.1.6 (2015-06-11)
------------------
* [jsk_footstep_planner] Enable roll paranoid mode
* Merge pull request #327 from garaemon/set-heuristic
  [jsk_footstep_planner] Add service interface to set heursitic function
* [jsk_footstep_planner] Add service interface to set heursitic function
* [jsk_footstep_controller] Fix parameters for jaxon red
* [jsk_footstep_planner] Fix for terrain task
* [jsk_footstep_planner] Add topic interface to project footprint
* [jsk_footstep_planner] Support JAXON_RED
* [jsk_footstep_planner, jsk_footstep_controller] Update for las vegas terrain with jaxon
* [jsk_footstep_planner] Enable roll paranoid mode again
* [jsk_footstep_planner, jsk_footstep_controller] Update for jaxon terrain
* [jsk_footstep_planner] Add simple script to transform frame_id of
  jsk_footstep_msgs/FootstepArray
* [jsk_footstep_controller] Change gait generator parameters according to
  plane condition including pitch angle and taking into account
  if transition is upward or downward
* [jsk_footstep_planner, jsk_footstep_controller] Compute x and y from
  previous coordinates to detect rolled plane
* [jsk_footstep_planner, jsk_footstep_controller] Support rolling terrain,
  I hope
* [jsk_footstep_planner, jsk_footstep_controller] Fix typo and update
  parameter for terrain
* [jsk_footstep_planner, jsk_footstep_controller] Support jaxon parameter files
* [jsk_footstep_planner, jsk_footstep_controller] Support jaxon footstep planning, Do not merge yet
* [jsk_footstep_planner] locally search reasonable goal when snapping
* [jsk_footstep_planner] Update offset parameter
* [jsk_footstep_planner] Support offset parameter from end coords to
  center of foot polygon
* [jsk_footstep_planner] Optimize function to remove shadow cells by using
  local coordinates
* [jsk_footstep_planner] Update projection parameter for slope terrain
* [jsk_footstep_planner] Add ~remove_shadow_cells to add padding to shadow cells
* [jsk_footstep_planner] Improve planning for different levels:
  1. Move successors a little bit when projecting footprint to different
  level
  2. Use footstep coordinate rather than mid-coords of the footstep as
  goal coordinates
* [jsk_footstep_planner] Check transition limit when planning across
  diffrent levels
* [jsk_footstep_planner] Add *gui-debug* symbol and ~toggle_gui_debug
  service to toggle debug using gui
* [jsk_footstep_planner] Add service API to project pose onto the nearest grid
* [jsk_footstep_planner] More greedy heuristic and compile euslisp code correctly
* [jsk_footstep_planner] Separate successors for same level and transition
  across different level
* [jsk_footstep_planner] Update successors' parameters
* [jsk_footstep_planner] Check range of grids first in occupancy-grid
* [jsk_footstep_planner] Use glVertexPointer and glDrawArrays to draw occupancy-grid
* [jsk_footstep_planner] Optimize creation of occupancy-grid by using
  integer-vector rather than hash-table
* [jsk_footstep_planner] Fix bug to compute rotate footstep to snap onto planes
* [jsk_footstep_planner] Update successors parameter
* [jsk_footstep_planner] add geo package to quaternion-from-two-vectors
* [jsk_footstep_planner] Read footstep parameter from file rather than ros parameter
* [jsk_footstep_planner] Use quaternion-from-two-vectors to compute rotate
  in project-coords-on-to-plane
* [jsk_footstep_planner] Add method to generate occupancy-grid from face
* [jsk_footstep_planner] Use mtimer instead of ros::time-now
* [jsk_footstep_planner] Fix minor bugs
* [jsk_footstep_planner] Memoize projecting grid and re-use it when the
  planner checks the plane is placable and refactor function names
* [jsk_footstep_planner] Optimize hash size according to cell num
* [jsk_footstep_planner] A lot of improvements on planner
  * Fix about orientation of projected footstep
  * Lazy evaluation to check if footprint is able to be on grid
  * Use mid-coords of footprints to evaluate heuristic
* [jsk_footstep_planner] Do not check if the footstep can be placable on
  plane across planes
* [jsk_footstep_planner] Support SimpleOccupancyGrid in footstep planner
* [jsk_footstep_planner] Snap geometry_msgs::PoseStamped (from rviz) onto grid map
* [jsk_footstep_planner] Move more functions to footstep_planner_utill.l
  from footstep_planner.l
* [jsk_footstep_planner] Support color in occupancy-grid class
* [jsk_footstep_planner] Return vertices in global coordinates in
  :vertices method of occupancy-grid
* [jsk_footstep_planner] Euslisp binding of
  jsk_recognition_msgs::SimpleOccupancyGrid message
* [jsk_footstep_planner] separate standalone utility functions into footstep_planner_util.l
* [jsk_footstep_planner] Add publisher of polygon of footprint for HRP2JSK
* [jsk_footstep_planner] Add euslisp wrapper to snap footstep on planes
* [jsk_footstep_plannar] Resolve pose of footstep respacted to initial footstep
* [jsk_footstep_plannner] Visualize euslisp footstep on rviz
* [jsk_footstep_planner] Use jsk_recognition_msgs
* Contributors: Ryohei Ueda, Yu Ohara

0.1.5 (2015-01-08)
------------------
* renamed make_sumple function
* added make-coords-list function
* added inverse_reachablity_with_given_coords
* Update drcmodel for current planner
* Add sample to compare heuristic functions
* add api to change successor
* Merge remote-tracking branch 'origin/master' into add-breakpoint-text
  Conflicts:
  jsk_footstep_controller/euslisp/footstep-controller.l
  jsk_footstep_controller/launch/hrp2jsknt_real_full.launch
* Add text publishing when checking breakpoint
* Do not allow step over 250mm stride
* Supress x-transition after z-transition. All the threshold is hard-coded
* Update footstep parameter for climing up stairs:
  larger footstep and smaller footprint
* Add dimensions of footsteps to the result of footstep planner
* Visualize footstep successors
* roseus only needs runtime
* Contributors: Kei Okada, Ryohei Ueda, Yu Ohara

0.1.4 (2014-10-21)
------------------

0.1.3 (2014-10-10)
------------------

0.1.2 (2014-09-08)
------------------

0.1.1 (2014-09-04)
------------------
* use lock/unlock service of environment server to lock/unlock the environment during planning
* compile euslisp file before running footstep planner
* publish footstep for visualization from planner
* update usage of env server according to the latest changeset of
  jsk_recognition
* use env server of jsk_pcl_ros
* ignore emtpy polygon message
* prepend initial steps to the result of the footstep planning
* call x::window-main-onw only if *debug* is t in jsk_footstep_planner/footstep-planner-node.l
* support 6dof planning
* adding model for footstep planning
* finalize footstep by goal steps
* supporting slope in footstep planning
* update for slope planning
* begins to support slope
* automatically choose the goal footstep
* store goal footstep to the problem class
* supress debug message of footstep planner
* update python scripts for catkin
* load msgs directory
* fix dependency
* keep permission of euslisp codes
* catkinize jsk_footstep_planner
* fix to keep orientation after projection to the planes
* supporting z-direction movement in planning
* supporting timeout of planning
* adding jsk_footstep_planner, euslisp implementation
* Contributors: Ryohei Ueda, Masaki Murooka
