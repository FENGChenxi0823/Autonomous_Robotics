
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();

}

kineval.buildFKTransforms = function buildFKTransforms(){
    traverseFKBase();
    traverseFKLink(robot.base);
}

function traverseFKBase(){
    var translation = generate_translation_matrix(robot.origin.xyz[0],robot.origin.xyz[1],robot.origin.xyz[2]);
    var rotation = generate_rotation_matrix_euler(robot.origin.rpy[0],robot.origin.rpy[1],robot.origin.rpy[2]);
    robot.links[robot.base].xform = matrix_multiply(translation, rotation);
    
    //add heading & lateral vector
    var heading_vector = [[0],[0],[1],[1]];
    robot_heading = matrix_multiply(robot.links[robot.base].xform,heading_vector);
    var lateral_vector = [[1],[0],[0],[1]];
    robot_lateral = matrix_multiply(robot.links[robot.base].xform,lateral_vector);


    if (robot.links_geom_imported) {
        robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2)));
    }

    return;
}



function traverseFKLink(currLink){
    if (robot.links[currLink].children == undefined) return;
    var joint;
    //for current link, traverse each its children joint
    for(joint of robot.links[currLink].children){
        var translation = generate_translation_matrix(robot.joints[joint].origin.xyz[0],robot.joints[joint].origin.xyz[1],robot.joints[joint].origin.xyz[2]);
        var rotation = generate_rotation_matrix_euler(robot.joints[joint].origin.rpy[0],robot.joints[joint].origin.rpy[1],robot.joints[joint].origin.rpy[2]);
        var transform = matrix_multiply(robot.links[currLink].xform, matrix_multiply(translation, rotation));

        //consider the motor DOF if links_geo_imported from ROS

        if (robot.joints[joint].type == "prismatic"){
            var joint_trans = [robot.joints[joint].angle * robot.joints[joint].axis[0],
                                robot.joints[joint].angle * robot.joints[joint].axis[1],
                                robot.joints[joint].angle * robot.joints[joint].axis[2]];
            transform = matrix_multiply(transform, generate_translation_matrix(joint_trans[0],joint_trans[1],joint_trans[2]));
        }
        else if((robot.joints[joint].type == "revolute") || (robot.joints[joint].type == "continuous") || (robot.joints[joint].type == undefined)){
            var q = quaternion_from_axisangle(robot.joints[joint].axis, robot.joints[joint].angle);
            var joint_rotate = generate_rotation_matrix_quaternion(quaternion_normalize(q));
            transform = matrix_multiply(transform, joint_rotate);
        }
        else{
            transform = matrix_multiply(transform, generate_identity(4));
        }

        robot.joints[joint].xform = matrix_copy(transform);
        traverseFKJoints(joint);
    }
    return;
}

function traverseFKJoints(currJoint){
    var link = robot.joints[currJoint].child;
    robot.links[link].xform = matrix_copy(robot.joints[currJoint].xform);
    traverseFKLink(link);
    return;
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

