"use strict";




function velocitySkinning(model)
{

    // update center of mass
    const N_joint = model["velocity_skinning"]["vertex_depending_on_joint"].length;

    for(let k_joint=0; k_joint<N_joint; ++k_joint){
        model["velocity_skinning"]["center_of_mass"][k_joint].set(0,0,0);
    }

    const com_temp = new THREE.Vector3();
    for(let k_joint=0; k_joint<N_joint; ++k_joint)
    {
        const vertex_dependency = model["velocity_skinning"]["vertex_depending_on_joint"][k_joint];
        const weight_dependency = model["velocity_skinning"]["vertex_weight_depending_on_joint"][k_joint];
        const N_dependency = vertex_dependency.length;

        for(let k_dep=0; k_dep<N_dependency; ++k_dep)
        {
            const idx_vertex = vertex_dependency[k_dep];
            const w_skinning = weight_dependency[k_dep];
            const p_vertex = model["vertex_skinning"][idx_vertex];

            com_temp.copy(p_vertex);
            com_temp.multiplyScalar(w_skinning);
            com_temp.divideScalar(N_dependency);

            model["velocity_skinning"]["center_of_mass"][k_joint].add(com_temp);
        }
    }


    for(let k=0, N=model["vertex_skinning"].length; k<N; ++k) {
        model["vertex_velocity_skinning"][k].copy(model["vertex_skinning"][k]);
        model["velocity_skinning_deformation"][k].set(0,0,0);
    }

    const S = new THREE.Matrix3();
    const R = new THREE.Matrix3();
    const Rt = new THREE.Matrix3();
    const T = new THREE.Matrix3();
    const deformation_flappy = new THREE.Vector3();
    const deformation_squashy = new THREE.Vector3();
    const ux = new THREE.Vector3().set(1,0,0);

    
    for(let k_joint=0; k_joint<N_joint; ++k_joint)
    {
        const p_joint = model["skeleton_current"]["position_global"][k_joint];
        const linear_speed = model["velocity_skinning"]["speed_tracker"][k_joint].current_speed;
        const angular_speed = model["velocity_skinning"]["rotation_tracker"][k_joint].current_speed;

        const angular_speed_norm = angular_speed.length();
        const linear_speed_norm = linear_speed.length();

        const vertex_dependency = model["velocity_skinning"]["vertex_depending_on_joint"][k_joint];
        const weight_dependency = model["velocity_skinning"]["vertex_weight_depending_on_joint"][k_joint];
        const center_of_mass = model["velocity_skinning"]["center_of_mass"][k_joint];

        const N_dependency = vertex_dependency.length;




        if(linear_speed_norm>1e-5)
        {
            const un_linear_speed = linear_speed.clone();
            un_linear_speed.divideScalar(linear_speed_norm);


            

            for(let k_dep=0; k_dep<N_dependency; ++k_dep)
            {
                // Flappy
                const idx_vertex = vertex_dependency[k_dep];
                const w_skinning = weight_dependency[k_dep];
                const p_vertex = model["vertex_skinning"][idx_vertex];

                const w_flappy = 0.4*model["velocity_skinning"]["flappy_weight"][idx_vertex]*model["param"]["flappy"];
                deformation_flappy.copy(linear_speed);
                deformation_flappy.multiplyScalar(-w_flappy);

                deformation_flappy.multiplyScalar(w_skinning);
                model["velocity_skinning_deformation"][idx_vertex].add(deformation_flappy);


                // Squashy
                const squash_factor = 0.2*linear_speed_norm*sceneElements.param["Squashy"]*model["param"]["squashy"];
                const elongation_scaling = 1.0+squash_factor;
                const squeeze_scaling = 1/Math.sqrt(1+squash_factor);

                S.elements[0] = elongation_scaling;
                S.elements[4] = squeeze_scaling;
                S.elements[8] = squeeze_scaling;
                
                rotation_between_vector(ux, un_linear_speed, R);
                Rt.copy(R);
                Rt.transpose();

                
                T.copy(R).multiply(S).multiply(Rt);
                deformation_squashy.copy(p_vertex).sub(center_of_mass);
                deformation_squashy.applyMatrix3(T);
                deformation_squashy.add(center_of_mass).sub(p_vertex);


                deformation_squashy.multiplyScalar(w_skinning);
                model["velocity_skinning_deformation"][idx_vertex].add(deformation_squashy);

            }

        }

        
        if(angular_speed_norm>1e-5)
        {
            const un_angular_speed = new THREE.Vector3().copy(angular_speed);
            un_angular_speed.divideScalar(angular_speed_norm);

            const u_joint_vertex = new THREE.Vector3();
            const vertex_speed = new THREE.Vector3();
            const rotation_center = new THREE.Vector3();

            for(let k_dep=0; k_dep<N_dependency; ++k_dep)
            {

                const idx_vertex = vertex_dependency[k_dep];
                const w_skinning = weight_dependency[k_dep];
                const p_vertex = model["vertex_skinning"][idx_vertex];
                
                
                u_joint_vertex.copy(p_vertex);
                u_joint_vertex.sub(p_joint);

                
                vertex_speed.copy(angular_speed);
                vertex_speed.cross(u_joint_vertex);
                const vertex_speed_norm = vertex_speed.length();


                rotation_center.copy(un_angular_speed);
                rotation_center.multiplyScalar( u_joint_vertex.dot(un_angular_speed)  );
                rotation_center.add(p_joint);

                const w_flappy = model["velocity_skinning"]["flappy_weight"][idx_vertex]*model["param"]["flappy"];

                const angle = - w_flappy * vertex_speed_norm * 0.4 * sceneElements.param['Flappy'];

                deformation_flappy.copy(p_vertex).sub(rotation_center);
                deformation_flappy.applyAxisAngle(un_angular_speed, angle);
                deformation_flappy.sub(p_vertex).add(rotation_center);

                deformation_flappy.multiplyScalar(w_skinning);
                
                
                model["velocity_skinning_deformation"][idx_vertex].add(deformation_flappy);
                

            }
        }

    }

    if(sceneElements.param["Activated"]===true){
        for(let k=0, N=model["vertex_skinning"].length; k<N; ++k) {
            model["vertex_velocity_skinning"][k].add(model["velocity_skinning_deformation"][k]);
        }
    }

}


let vec1_temp = new THREE.Vector3();
let vec2_temp = new THREE.Vector3();
function rotation_between_vector(vec1, vec2, R)
{
    vec1_temp.copy(vec1);
    vec1_temp.sub(vec2);

    if( vec1_temp.lengthSq() < 1e-6){
        R.set(1,0,0, 0,1,0, 0,0,1);
        return ;
    }

    const c = vec1.dot(vec2);
    const s = Math.sqrt(1-c*c);
    const axis = vec1_temp;
    axis.copy(vec1).cross(vec2);
    axis.normalize();

    const x = axis.x;
    const y = axis.y;
    const z = axis.z;
    R.set(c+x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s,
    y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s,
    z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c));
}