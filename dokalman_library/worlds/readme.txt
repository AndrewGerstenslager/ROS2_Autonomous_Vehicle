Getting Started:
1. run command in terminal: export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/{insert path here}/models
2. run just the world by using: gazebo --verbose ~/{insert path here}/worlds/line_det_course.world
*not sure how to spawn the robot model in the world just yet*

To change the ground texture:
1. Add png file to the models/line_det_course/materials/textures folder
2. Change the ground.png in materials/scripts/ground.material
3. Change the ground.png in line_det_course/model.sdf
*alternatively you could just replace the ground.png file in the textures folder with a new one with the same name.
