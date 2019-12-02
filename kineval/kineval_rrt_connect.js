
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for(i = 0; i < q_goal_config.length; i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    treeA = tree_init(q_start_config);
    treeB = tree_init(q_goal_config);
    // console.log("inittreeA : ", treeA.vertices[0].vertex);
    // console.log("inittreeB : ", treeB.vertices[0].vertex);
    goal_idx = 0;
    step = 0.2;
    find_valid_path = false;
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 2;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
        if (rrt_alg !=1 && Math.random() < 0.2)
            var q_rand = q_goal_config;
        else var q_rand = random_config();

        if(rrt_alg == 0){
            rrt_iter_count ++;
            extend = rrt_extend(treeA, q_rand)[0];
            console.log(extend, rrt_iter_count);
            if (extend == "reached") find_path();
            return extend;
        }
        else if(rrt_alg == 1){
            var extend = rrt_extend(treeA, q_rand);
            if(extend[0] != "trapped"){
                var q_target = extend[1];
                if (rrt_connect(treeB, q_target) == "reached"){
                    find_path();
                    return "reached";
                }
            }
            tempTree= treeA;
            treeA =  treeB;
            treeB = tempTree;
            // console.log("treeA : ", treeA.vertices[0].vertex);
            // console.log("treeB : ", treeB.vertices[0].vertex);
            rrt_iter_count ++;
            return "iterating";
        }
        else if(rrt_alg == 2){
            rrt_iter_count ++;
            extend = rrtstar_extend(treeA, q_rand);
            console.log("rrtstar: ", rrt_iter_count);
            if (!find_valid_path && extend[0] == "reached") {
                find_valid_path = true;
                goal_idx = extend[1];
                find_path();
            }
            if(find_valid_path  && rrt_iter_count > 2000){
                find_improved_path(goal_idx);
                console.log("finished");
                return "reached";
            }
        }
    }
    return "failed";
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].cost = 0;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;
    new_vertex.cost = 0;
    new_vertex.visited = false;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

function random_config(){
    var q_rand_config = new Array(q_start_config.length);
    
    var upper_bound;
    var lower_bound;
    for(i = 0; i < q_rand_config.length; i++){
        //set the robot_boundary
        if (i < 3){
            lower_bound = robot_boundary[0][i];
            upper_bound = robot_boundary[1][i];
        }else if(i == 3 || i == 5){
            lower_bound = 0;
            upper_bound = 0;
        }else if(i == 4){
            lower_bound = -Math.PI;
            upper_bound = Math.PI;
        }else{
            if (robot.joints[q_index[i]].type == 'prismatic'|| robot.joints[q_index[i]].type =='revolute'){
                lower_bound = robot.joints[q_index[i]].limit.lower;
                upper_bound = robot.joints[q_index[i]].limit.upper;
            }else if (robot.joints[q_index[i]].type == 'continuous'){
                lower_bound = -Math.PI;
                upper_bound = Math.PI;
            }else {
                lower_bound = 0;
                upper_bound = 0;
            }
        }
        q_rand_config[i] = lower_bound + (upper_bound-lower_bound) * Math.random();
    } 
    
    return q_rand_config;
}

function rrt_extend(tree, q_rand){
    var extend_status = "trapped";
    var q_near = nearest_neighbor(tree, q_rand);
    // console.log(q_near[0], q_near[1], q_rand);
    var q_new = new_config(q_near[0], q_rand);
    // console.log("q_new : ", q_new);
    if(!kineval.poseIsCollision(q_new)){
        tree_add_vertex(tree, q_new);
        tree_add_edge(tree, q_near[1], tree.newest);
        if(isGoal(q_new, q_goal_config)) extend_status = "reached";
        else if(isGoal(q_new, q_rand)){
            extend_status = "connected";
        }else{
            extend_status = "advanced";
        }
    }
    return [extend_status, q_new];

}

function rrt_connect(tree, q_target){
    var connect_status = "advanced";
    while(connect_status == "advanced"){
        connect_status = rrt_extend(tree, q_target)[0];
    }
    if(connect_status == "connected" || connect_status == "reached") 
    return "reached";
}


function rrtstar_extend(tree, q_rand){
    var extend_status = "trapped";
    var goal_idx = 0;
    var q_near = nearest_neighbor(tree, q_rand);
    var q_new = new_config(q_near[0], q_rand);
    console.log(q_near[0],q_rand, q_new);
    if(!kineval.poseIsCollision(q_new)){
        tree_add_vertex(tree, q_new);
        var x_near = nearVertices(tree,q_new);
        var q_min = findParent(tree, x_near, q_new); 
        tree_add_edge(tree, q_min[1], tree.newest);
        rewire(tree, x_near,q_min, q_new);
        if(isGoal(q_new, q_goal_config)) {
            extend_status = "reached";
            goal_idx = tree.newest;
        }
        else extend_status = "advanced";
    }
    return [extend_status, goal_idx];

}

function nearVertices(tree, q_new){
    var x_near = [];
    var r = 0.3;
    for(var i =0; i <tree.vertices.length-1; i++){
        var d = vector_norm(vector_minus(tree.vertices[i].vertex, q_new));
        if(d < r){
            x_near.push([tree.vertices[i], i]);
        }
    }
    // console.log("nearVertices: ", x_near.length);
    return x_near;
}


function findParent(tree, x_near, q_new){
    var min_d = Infinity;
    var parent_idx = 0;
    var parent_config;
    for(var i= 0; i < x_near.length; i++){
        var d = x_near[i][0].cost + vector_norm(vector_minus(x_near[i][0].vertex, q_new));
        if (d < min_d) {
            min_d = d;
            parent_config = x_near[i][0];
            parent_idx = x_near[i][1];
        }
    }
    // console.log("findParent: ", parent_config, parent_idx, min_d);
    tree.vertices[tree.newest].cost = min_d;
    return [parent_config, parent_idx];
}

function rewire(tree, x_near,q_min, q_new){
    for(var i = 0; i<x_near.length; i++){
        var new_cost = tree.vertices[tree.newest].cost + vector_norm(vector_minus(x_near[i][0].vertex, q_new));
        if (x_near[i][1] != q_min[1] && x_near[i][0].cost > new_cost){
            //change parent
            x_near[i][0].edges.shift();
            x_near[i][0].edges.unshift(tree.vertices[tree.newest]);
            x_near[i][0].cost = new_cost;
        }
    }

}

function isGoal(q_new, q_goal){
    // var reached = true;
    // for(var i =0; i < q_goal.length; i++){
    //     if(Math.abs(q_new[i] - q_goal[i]) > step/2){
    //         reached = false;
    //         break;
    //     }
    // }
    // return reached;
    // console.log (q_new, q_goal,vector_norm(vector_minus(q_new,q_goal)));
    return (vector_norm(vector_minus(q_new,q_goal)) < step/2);


}

function nearest_neighbor(tree, q_rand){
    var min_dis = Infinity;
    var q_near;
    var idx;
    for(var i =0; i <tree.vertices.length; i++){

        var d = vector_norm(vector_minus(tree.vertices[i].vertex, q_rand));
        if(d < min_dis){
            min_dis = d;
            q_near = tree.vertices[i].vertex;
            idx =i;
        }
    }
    return [q_near, idx];
}


function new_config(q1, q2){
    // console.log(q1,q2);
    if (isSame(q1,q2)) return q1;
    var direction = vector_normalize(vector_minus(q2,q1));
    // console.log(direction);
    var q_new = [];
    for(var i = 0; i< q1.length; i++){
        q_new[i] = q1[i] + step* direction[i];
    }
    return q_new;
}


function find_path(){
    console.log("find path");
    kineval.motion_plan.length = 0;
    // if (rrt_alg == 1){
    var pathA = path_trace(treeA);
    var pathB = path_trace(treeB);
    // console.log("pathB found"); 
    //push start path and then the goal path 
    if(isGoal(pathA[0].vertex, q_goal_config)){
        //push pathB first and then path A
        for(var i =0; i < pathB.length; i++){
            kineval.motion_plan.push(pathB[i]);
        }
        for(var j = pathA.length-1; j >= 0; j--){
            kineval.motion_plan.push(pathA[j]);
        }
    //if tree A is goal tree
    }
    else{
        for(var i =0; i < pathA.length; i++){
            kineval.motion_plan.push(pathA[i]);
        }
        for(var j = pathB.length-1; j >= 0; j--){
            kineval.motion_plan.push(pathB[j]);
        }
    }//if treeB is goal tree
    // }else if(rrt_alg ==0 || rrt_alg ==2){
    //     var path = path_dfs(treeA);
    //     console.log("pathA found");
    //     for(var i =0; i < path.length; i++){
    //         kineval.motion_plan.push(path[i]);
    //     }
    // }
}

function path_trace(tree){
    var path = [];
    var v = tree.vertices[tree.newest];

    // console.log(tree.newest, tree.vertices[tree.newest]);
    while(!isSame(v.vertex, tree.vertices[0].vertex)){
        path.unshift(v);
        v.geom.material.color = {r:1,g:0,b:0};
        v = v.edges[0];
        // console.log(v.vertex);
    }
    path.unshift(tree.vertices[0]);
    return path;
}



function find_improved_path(goal_idx){
    console.log("rrtstar find improved path");    
    var path = path_dfs(treeA, goal_idx);
    for(var i =0; i < path.length; i++){
        kineval.motion_plan.push(path[i]);
    }

}

function path_dfs(tree,goal_idx){
    var path = [];
    // var goal;
    // var found = false;
    // var stack = [];
    // stack.push(tree.vertices[0]);
    // tree.vertices[0].visited = true;

    // while(stack.length != 0 && !found){
    //     var curr_v = stack.pop();
    //     var children = curr_v.edges;
    //     for(var v of children){
    //         console.log(children.length, v.vertex);
    //         if (isSame(v.vertex, q_goal_config)){
    //             goal = v;
    //             console.log("find goal: ", goal, goal.vertex);
    //             found = true;
    //             break;
    //         }
    //         if (v.visited == false) {
    //             stack.push(v);
    //             v.visited = true;
    //         }
    //     }
    // }
    
    var dummy_v = tree.vertices[goal_idx];


    while(!isSame(dummy_v.vertex, tree.vertices[0].vertex)){
        path.unshift(dummy_v);
        dummy_v.geom.material.color = {r:0,g:1,b:0};
        dummy_v = dummy_v.edges[0];
        // console.log(v.vertex);
    }
    path.unshift(tree.vertices[0]);
    return path;
}

function isSame(v1,v2){
    var same = true;
    for(var i =0; i < v1.length; i++){
        if(v1[i] != v2[i]){
            same = false;
            break;
        }
    }
    return same;
}












