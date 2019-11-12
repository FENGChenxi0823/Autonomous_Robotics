
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
  // get endeffector Cartesian position in the world
  endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);
  // compute distance of endeffector to target
  kineval.params.trial_ik_random.distance_current = Math.sqrt(
          Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
          + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
          + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );
  // if target reached, increment scoring and generate new target location
  // KE 2 : convert hardcoded constants into proper parameters
  if (kineval.params.trial_ik_random.distance_current < 0.01) {
      kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
      kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
      kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
      kineval.params.trial_ik_random.targets += 1;
      textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
  }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    

    var Jacobian_t = [];

    var current_joint = endeffector_joint;
    var tool_position = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    var tool_orien = rot2rpy(robot.joints[endeffector_joint].xform);
    var endeffector_actual_world = {};
    endeffector_actual_world.position = [tool_position[0][0],tool_position[1][0],tool_position[2][0],tool_position[3][0]];
    endeffector_actual_world.orientation = tool_orien;
    // console.log("actual ", endeffector_actual_world.position,endeffector_actual_world.orientation);
    // console.log("target ", endeffector_target_world.position,endeffector_target_world.orientation);

    var endeffector_position_error = vector_minus(endeffector_target_world.position, endeffector_actual_world.position);
    var endeffector_orientation_error =[];
    if(kineval.params.ik_orientation_included){
      endeffector_orientation_error = vector_minus(endeffector_target_world.orientation, endeffector_actual_world.orientation);
    }else{
      endeffector_orientation_error = [0,0,0];
    }
    
    // console.log("error ", endeffector_position_error,endeffector_orientation_error);
    var endeffector_error=[];
    for(var i = 0; i < 6; i++){
      if (i < 3)
        endeffector_error[i] = [endeffector_position_error[i]];
      else
        endeffector_error[i] = [endeffector_orientation_error[i-3]];
    }
    // console.log(endeffector_error);

    var joint_idx = 0;

    // calculate Jacobian matrix
    while(true){
        //calculate the Jacobian for this joint
        var joint_axis = matrix_multiply_vector(matrix_invert_affine(robot.joints[current_joint].xform), robot.joints[current_joint].axis);
        var joint_position = matrix_multiply(robot.joints[current_joint].xform, [[0],[0],[0],[1]]);
        joint_position = [joint_position[0][0],joint_position[1][0],joint_position[2][0]];
        var w = joint_axis;
        // the fixed joint's angle will be zero so that jacobian doesnt' matter  here
        if(robot.joints[current_joint].type == "prismatic"){
            Jacobian_t[joint_idx] = [w[0],w[1],w[2],0,0,0];
        }
        else{
          var r = vector_minus([tool_position[0][0],tool_position[1][0],tool_position[2][0]], joint_position);
          var rot = vector_cross(w,r);
          Jacobian_t[joint_idx] = [rot[0],rot[1],rot[2],w[0],w[1],w[2]];
        }

        current_link = robot.joints[current_joint].parent;
        if(current_link == robot.base) break;
        current_joint = robot.links[current_link].parent;
        joint_idx++;

    }
    // console.log("endeffector_joint xform ", robot.joints[endeffector_joint].xform);
    //calculate control
    // console.log(Jacobian_t);
    var Jacobian = matrix_transpose(Jacobian_t);
    var control = [];
    if (!kineval.params.ik_pseudoinverse){
         control = matrix_multiply(Jacobian_t, endeffector_error);
    }
    else{
         control = matrix_multiply(matrix_pseudoinverse(Jacobian), endeffector_error);
    }
    // console.log(Jacobian_t);
    // console.log(control);
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



