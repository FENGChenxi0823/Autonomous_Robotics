
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    

    var Jacobian_t = [];

    var current_joint = endeffector_joint;

    var tool_position = matrix_multiply_vector(robot.joints[endeffector_joint].xform, endeffector_position_local);
    var tool_orien = rot2rpy(robot.joints[endeffector_joint].xform);
    var endeffector_actual_world = tool_position.concate(tool_orien);

    var endeffector_position_error = vector_minus(endeffector_target_world,endeffector_actual_world);

    var joint_idx = 0;

    //calculate Jacobian matrix
    while(true){
        //calculate the Jacobian for this joint
        var joint_axis = matrix_multiply_vector(robot.joints[current_joint].xform,robot.joints[current_joint].axis);
        var joint_position = matrix_multiply_vector(robot.joints[current_joint].xform, [[0],[0],[0],[1]]);
        var w = vector_minus(joint_axis - joint_position);

        // the fixed joint's angle will be zero so that jacobian doesnt' matter  here
        if(robot.joints[current_joint].type == "prismatic"){
            Jacobian_t[joint_idx] = [w[0],w[1],w[2],0,0,0];
        }
        else{
            var r = tool_position - joint_position;
            var rot = vector_cross(w,r);
            Jacobian[joint_idx] = [rot[0],rot[1],rot[2],w[0],w[1],w[2]];
        }

        current_link = robot.joints[current_joint].parent;
        if(current_link == robot.base) break;
        current_joint = robot.links[current_link].parent;
        joint_idx++;

    }

    //calculate control
    var Jacobian = matrix_transpose(Jacobian_t);
    var control = [];
    if (!kineval.params.ik_pseudoinverse){
         control = matrix_multiply(Jacobian_t, endeffector_position_error);
    }
    else{
         control = matrix_multiply(matrix_pseudoinverse(Jacobian), endeffector_position_error);
    }

    //apply control to each joint
    current_joint = endeffector_joint;
    joint_idx = 0;
    while(true){
        robot.joints[current_joint].control += kineval.params.ik_steplength * control[joint_idx];
        current_link = robot.joints[current_joint].parent;
        if(current_link == robot.base) break;
        current_joint = robot.links[current_link].parent;
        joint_idx++;

    }
}



