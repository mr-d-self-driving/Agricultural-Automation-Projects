# Agricultural-Automation-Projects
Projects for the course EBS 289k: Sensors and Actuators in Agricultural Automation
Author: Rayne Milner, Stavros Vougioukas

Contact: rmmilner@ucdavis.edu

Files: 

FinalProject.m: Masterfile for project. Initializes parameters, calls all functions, performs C.V. algorithms and produces final output.

createPath.m: Path planning script, relies on genetic algorithm to optomize route through orchard and create shortest dubins path.

PurePursuitController.m: Path tracking script, uses pure pursuit algorithm.

tspof_ga.m: AUTHOR: Joseph Kirk. Solves traveling salesman problem to compute optimal path through orchar using genetic algorithm.

XYtoIJ.m: Performs transform from world to robot frame.

updateLaserBeamGrid.m: Updates nursery occupancy grid.

updateLaserBeamBitmap.m: Updates nursery bitmap.

robotOdo.m: AUTHOR: Stavros Vougioukas. Produces simulated noisy odometery data.

plotTractor.m: Plots tractor model.

LaserScannerNoisy.p: AUTHOR: Stavros Vougioukas. Produces simulated noisy LIDAR data.

laserRange.m: Takes noisy LIDAR data and performs range finding algorithm.

KinematicModel.m: Performs euler integration to find final state of tractor.

generateNursery.m: creates a randomized sinmulated nursery for testing.

errorcalc.m: performs statistical analysis on output data.

draw_disc.m: Draws simulated nursery.

bresenhamfast.m: AUTHOR:Peter I. Corke. Performs Bresenham line algorithm 


