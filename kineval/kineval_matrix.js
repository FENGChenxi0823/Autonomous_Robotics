//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

function matrix_multiply(m1,m2){
    var mat =[];
    var i,j,k;

    for (i=0; i<m1.length;i++){
        mat[i] = [];
        for(j=0; j<m2[0].length; j++){
            var temp =0;
            for(k = 0; k<m2.length; k++){
                temp += m1[i][k]*m2[k][j];
            }
            mat[i][j] = temp;
        }
    }
    return mat;

}

function matrix_transpose(m1){
    var mat = [];
    var i, j;

    for(i=0; i<m1[0].length; i++){
        mat[i] =[];
        for(j=0; i<m1.length;j++){
            mat[i][j] =m1[j][i];
        }
    }

    return mat;
}

function matrix_pseudoinverse(m1){
    var N = m1.length;
    var M = m1[0].length;
    if(N>M){
        var m = numeric.inv(matrix_multiply(matrix_tanspose(m1), m1));
        return matrix_multiply(m, matrix_tanspose(m1)); 
    }
    else if(N<M){
        var m = numeric.inv(matrix_multiply(m1, matrix_tanspose(m1)));
        return matrix_multiply(matrix_tanspose(m1), m);
    }else{
        return numeric.inv(m1);
    }
}

// function matrix_invert_affine(){

// }


function vector_normalize(v1){
    var vec = [];
    var i;
    var norm = Math.sqrt(v1[0]**2 + v1[1]**2 +v1[2]**2);
    for(i = 0; i<v1.length; i++){
        vec[i] = v1[i]/norm;
    }
    return vec;
}

function vector_cross(v1,v2){
    var vec = [];
    vec[0] = v1[1]*v2[2] - v1[2]*v2[1];
    vec[1] = - ( v1[0]*v2[2] - v1[2]*v2[0] );
    vec[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return vec;

}

function  generate_identity(){
    var m = [];
    var k=4;
    for (i=0;i<k;++i){
        m[i] = [];
        for (j=0;j<k;++j){
            if (i!=j){
                m[i][j] = 0;
            } else {
                m[i][j] = 1;
            }
        }
    }
    return m;
}

function   generate_translation_matrix(x,y,z){
    var m = generate_identity(4);
    m[0][3] = x;
    m[1][3] = y;
    m[2][3] = z;
    return m;
}

function   generate_rotation_matrix_X(angle){
    var m = generate_identity(4);
    m[1][1] = Math.cos(angle);
    m[2][2] = Math.cos(angle);
    m[1][2] = - Math.sin(angle);
    m[2][1] = Math.sin(angle);
    return m;  
}

function   generate_rotation_matrix_Y(angle){
    var m = generate_identity(4);
    m[0][0] = Math.cos(angle);
    m[2][2] = Math.cos(angle);
    m[0][2] = Math.sin(angle);
    m[2][0] = - Math.sin(angle);
    return m;  
}

function   generate_rotation_matrix_Z(angle){
    var m = generate_identity(4);
    m[0][0] = Math.cos(angle);
    m[1][1] = Math.cos(angle);
    m[0][1] = - Math.sin(angle);
    m[1][0] = Math.sin(angle);
    return m;  
}

function generate_rotation_matrix_euler(r,p,y){
    var m = matrix_multiply(generate_rotation_matrix_Z(y),matrix_multiply(generate_rotation_matrix_Y(p), generate_rotation_matrix_X(r)));
    return m;
}


    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

