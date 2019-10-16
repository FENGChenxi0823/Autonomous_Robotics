robot = {
  name:"ur10_chenxif", 
  base:"base_link", 
  origin:{ xyz: [0,0.1,0], rpy:[0,0,0] },
  links: {
    "base_link": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "base.dae" } },
        material : { color : { rgba : [0.7, 0.7, 0.7, 1.0] } }
      }
    },
    "shoulder_link": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "shoulder.dae" } },
        material : { color : { rgba : [0.7, 0.7, 0.7, 1.0] } }
      }
    },
    "upper_arm_link": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "upperarm.dae" } },
        material : { color : { rgba : [0.7, 0.7, 0.7, 1.0] } }
      }
    },
    "forearm_link": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "forearm.dae" } },
        material : { color : { rgba : [0.7, 0.7, 0.7, 1.0] } }
      }
    },
    "wrist_1_link": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "wrist1.dae" } },
        material : { color : { rgba : [0.7, 0.7, 0.7, 1.0] } }
      }
    },
    "wrist_2_link": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "wrist2.dae" } },
        material : { color : { rgba : [0.7, 0.7, 0.7, 1.0] } }
      }
    },
    "wrist_3_link": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "wrist3.dae" } },
        material : { color : { rgba : [0.7, 0.7, 0.7, 1.0] } }
      }
    },
  },
};

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "wrist_3_joint";
robot.endeffector.position = [ [0.1],[0],[0],[1] ]

robot.joints = {};

robot.joints.shoulder_pan_joint = {parent:"base_link", child:"shoulder_link"};
robot.joints.shoulder_pan_joint.axis = [0,0,1];
robot.joints.shoulder_pan_joint.type = "revolute";
robot.joints.shoulder_pan_joint.origin = {xyz: [0,0,0.1273], rpy:[0,0,0]};
robot.joints.shoulder_pan_joint.limit = {lower:-6.2832, upper:6.2832};

robot.joints.shoulder_lift_joint = {parent:"shoulder_link", child:"upper_arm_link"};
robot.joints.shoulder_lift_joint.axis = [0,1,0];
robot.joints.shoulder_lift_joint.type = "revolute";
robot.joints.shoulder_lift_joint.origin = {xyz: [0,0.220941,0], rpy:[0, 1.5708, 0]};
robot.joints.shoulder_lift_joint.limit = {lower:-6.2832, upper:6.2832};

robot.joints.elbow_joint = {parent:"upper_arm_link", child:"forearm_link"};
robot.joints.elbow_joint.axis = [0,1,0];
robot.joints.elbow_joint.type = "revolute";
robot.joints.elbow_joint.origin = {xyz: [0,-0.1719,0.5723], rpy:[0, 0, 0]};
robot.joints.elbow_joint.limit = {lower:-3.1416, upper:3.1416};

robot.joints.wrist_1_joint = {parent:"forearm_link", child:"wrist_1_link"};
robot.joints.wrist_1_joint.axis = [0,1,0];
robot.joints.wrist_1_joint.type = "revolute";
robot.joints.wrist_1_joint.origin = {xyz: [0,0,0.5723], rpy:[0, 1.5708, 0]};
robot.joints.wrist_1_joint.limit = {lower:-6.2832, upper:6.2832};

robot.joints.wrist_2_joint = {parent:"wrist_1_link", child:"wrist_2_link"};
robot.joints.wrist_2_joint.axis = [0,0,1];
robot.joints.wrist_2_joint.type = "revolute";
robot.joints.wrist_2_joint.origin = {xyz: [0,0.1149,0], rpy:[0, 0, 0]};
robot.joints.wrist_2_joint.limit = {lower:-6.2832, upper:6.2832};

robot.joints.wrist_3_joint = {parent:"wrist_2_link", child:"wrist_3_link"};
robot.joints.wrist_3_joint.axis = [0,1,0];
robot.joints.wrist_3_joint.type = "revolute";
robot.joints.wrist_3_joint.origin = {xyz: [0,0,0.1157], rpy:[0, 0, 0]};
robot.joints.wrist_3_joint.limit = {lower:-6.2832, upper:6.2832};

robot.links_geom_imported = true;

links_geom = {};

  // KE: replace hardcoded robot directory
  // KE: replace file extension processing
progressLinkLoading = 0;
i = 0;
imax = Object.keys(robot.links).length;
for (x in robot.links) {
  //geom_index = robot.links[x].visual.geometry.mesh.filename.split('_adjusted')[0];
  //geom_extension = robot.links[x].visual.geometry.mesh.filename.split('_adjusted')[1];
  filename_split = robot.links[x].visual.geometry.mesh.filename.split('.');
  geom_index = filename_split[0];
  geom_extension = filename_split[filename_split.length-1];
  console.log(geom_index + "  " + geom_extension);
  //assignFetchModel('./robots/fetch/'+robot.links[x].visual.geometry.mesh.filename,geom_index);
  if (geom_extension === "dae") { // KE: extend to use regex
    assignFetchModelCollada('./robots/ur10_chenxif/meshes/'+robot.links[x].visual.geometry.mesh.filename,x);
  }
  else if (geom_extension === "DAE") { // extend to use regex
    assignFetchModelCollada('./robots/ur10_chenxif/meshes/'+robot.links[x].visual.geometry.mesh.filename,x);
  }
  else {
    assignFetchModelSTL('./robots/ur10_chenxif/meshes/'+robot.links[x].visual.geometry.mesh.filename,robot.links[x].visual.material,x);
  }
  i++;

  progressLinkLoading = i/imax; 
  console.log("Robot geometry: progressLinkLoading " + progressLinkLoading*100);
}

function assignFetchModelCollada(filename,index) {

    console.log("assignFetchModel : "+filename+" - "+index); 
    var collada_loader = new THREE.ColladaLoader();
    var val = collada_loader.load(filename, 
       function ( collada ) {
            links_geom[index] = collada.scene;
        },
        function (xhr) {
            console.log(filename+" - "+index+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        }
    );
}

function assignFetchModelCollada2(filename,index) {

    console.log("assignFetchModel : "+filename+" - "+index); 
    var collada_loader = new ColladaLoader2();
    var val = collada_loader.load(filename, 
       function ( collada ) {
            links_geom[index] = collada.scene;
        },
        function (xhr) {
            console.log(filename+" - "+index+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        }
    );
}


function assignFetchModelSTL(filename,material_urdf,linkname) {

    console.log("assignFetchModel : "+filename+" - "+linkname); 
    var stl_loader = new THREE.STLLoader();
    var val = stl_loader.load(filename, 
       function ( geometry ) {
            // ocj: add transparency
            var material_color = new THREE.Color(material_urdf.color.rgba[0], material_urdf.color.rgba[1], material_urdf.color.rgba[2]);
            var material = new THREE.MeshLambertMaterial( {color: material_color, side: THREE.DoubleSide} );
            links_geom[linkname] = new THREE.Mesh( geometry, material ) ;
        } //,
        //function (xhr) {
        //    console.log(filename+" - "+linkname+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        //}
    );
}


