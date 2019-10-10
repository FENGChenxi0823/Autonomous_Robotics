//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

function quaternion_from_axisangle(axis, angle){
	var axis_norm = Math.sqrt(axis[0]**2 + axis[1]**2 +axis[2]**2);
	axis = [axis[0]/axis_norm, axis[1]/axis_norm, axis[2]/axis_norm];
	return [Math.cos(angle/2), axis[0]*Math.sin(angle/2), axis[1]*Math.sin(angle/2), axis[2]*Math.sin(angle/2)]
}

function quaternion_normalize(q){
	var q_norm = Math.sqrt(q[0]**2 + q[1]**2+ q[2]**2 + q[3]**2);
	return [q[0]/q_norm, q[1]/q_norm, q[2]/q_norm, q[3]/q_norm ];
}


function generate_rotation_matrix_quaternion(q){
	var m = generate_identity(4);
    m[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    m[0][1] = 2*( q[1]*q[2] - q[0]*q[3] );
    m[0][2] = 2*( q[0]*q[2] + q[1]*q[3] );
    m[1][0] = 2*( q[1]*q[2] + q[0]*q[3] );
    m[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    m[1][2] = 2*( q[3]*q[2] - q[0]*q[1] );
    m[2][0] = 2*( q[1]*q[3] - q[0]*q[2] );
    m[2][1] = 2*( q[1]*q[0] + q[2]*q[3] );
    m[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    return m;
}


function quaternion_multiply(q1,q2){
    var q = new Array(q1.length);
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q[3] = q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] + q1[3]*q2[0];
    return q;
}
