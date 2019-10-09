
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
    // console.table(rotation);
    return;
}

function traverseFKLink(currLink){
    if (robot.links[currLink].children == undefined) return;
    // console.log("currLink "+ currLink +" children joints " + robot.links[currLink].children);
    var joint;
    for(joint of robot.links[currLink].children){
        console.log("currLink"+ currLink +" joint " + joint);
        var translation = generate_translation_matrix(robot.joints[joint].origin.xyz[0],robot.joints[joint].origin.xyz[1],robot.joints[joint].origin.xyz[2]);
        var rotation = generate_rotation_matrix_euler(robot.joints[joint].origin.rpy[0],robot.joints[joint].origin.rpy[1],robot.joints[joint].origin.rpy[2]);
        var transform = matrix_multiply(translation, rotation);
        robot.joints[joint].xform = matrix_multiply(robot.links[currLink].xform, transform);
        console.table(robot.joints[joint].xform);
        traverseFKJoints(joint);
    }
    return;
}

function traverseFKJoints(currJoint){
    var link = robot.joints[currJoint].child;
    console.log("currJoint: "+ currJoint +" link " + link);
    robot.links[link].xform = robot.joints[currJoint].xform;
    // console.table(robot.lin ks[link].xform);
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

