#include "skinning_loader.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include <fstream>
#include <sstream>
#include "quaternion.hpp"

using namespace vcl;

#ifdef _WIN32
#pragma warning(disable : 4267)
#endif

struct parameters_basic_loader
{
    std::string common_dir;
    GLuint shader;
    float scaling = 1.0f;
};


void load_sphere_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);      // no rotation

    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0};
    joint_geometry g1   = {{0.0f,0.0f,-1.0f},q0};
    joint_geometry g2   = {{0.0f,0.0f,0.0f},q0};

    // Skeleton at rest shape
    skeleton.rest_pose = { g0 };

    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g1},{2,g2}};

    skeleton.anim = { anim_g0 };


    // Cylinder shape
    mesh sphere;
    const size_t N=25;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = float(3.14f)* v;
            const float phi = 2*float(3.14f)* u;

            const float x = r * std::sin(theta) * std::cos(phi);
            const float y = r * std::sin(theta) * std::sin(phi);
            const float z = r * std::cos(theta);
            const vec3 p = {x,y,z};


            const vec3 n = p / norm(p);

            sphere.position.push_back(p);
            sphere.normal.push_back(n);
            sphere.texture_uv.push_back({u,v});
        }
    }
    sphere.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        //const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            skinning.influence.push_back( {{0, 1.0f}} );
        }
    }

    weight_flappy.resize(sphere.position.size());
    weight_flappy.fill(0.0f);


    skinning.rest_pose = sphere.position;
    skinning.rest_pose_normal = sphere.normal;
    skinning.deformed  = sphere;

    skinning.deformed.fill_empty_fields();


    shape_visual.clear();
    shape_visual = mesh_drawable(sphere);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}

void load_rondinella_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy,vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);      // no rotation

    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0};
    joint_geometry g1   = {{0.0f,0.0f,-1.0f},q0};
    joint_geometry g2   = {{0.0f,0.0f,0.0f},q0};

    // Skeleton at rest shape
    skeleton.rest_pose = { g0 };

    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g1},{2,g2}};

    skeleton.anim = { anim_g0 };


    // Cylinder shape
    mesh sphere;
    sphere = mesh_load_file_obj("assets/rondinella/rondinella2.obj");

    for(int k=0; k<int(sphere.position.size()); ++k)
        sphere.position[k] /= 5.0f;

    mat3 R = rotation_from_axis_angle_mat3({1,0,0}, -3.14f/2.0f);
    for(int k=0; k<int(sphere.position.size()); ++k)
        sphere.position[k] = R*sphere.position[k];

    sphere.fill_empty_fields();

//    const size_t N=50;
//    const float r = 0.1f;
//    skinning.influence.clear();
//    for(size_t ku=0; ku<N; ++ku)
//    {
//        for(size_t kv=0; kv<N; ++kv)
//        {
//            const float u = ku/float(N-1.0f);
//            const float v = kv/float(N);

//            const float theta = float(M_PI)* v;
//            const float phi = 2*float(M_PI)* u;

//            const float x = r * std::sin(theta) * std::cos(phi);
//            const float y = r * std::sin(theta) * std::sin(phi);
//            const float z = r * std::cos(theta);
//            const vec3 p = {x,y,z};

//            const vec3 n = p / norm(p);

//            sphere.position.push_back(p);
//            sphere.normal.push_back(n);
//        }
//    }
//    sphere.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    int const N = sphere.position.size();
    for(int k=0; k<N; ++k)
    {
        skinning.influence.push_back( {{0, 1.0f}} );
    }

    weight_flappy.resize(0);
    weight_squashy.resize(0);
    for(int k=0; k<N; ++k)
    {
        const vec3& p = sphere.position[k];

        const vec3& pc = p-vec3(0,0.05f,0);


        float distance = norm(pc);



        float w_flappy = 0.0f;
        float w_squashy = 0.0f;

        float threshold = 0.25f;
        if(distance<threshold) {
            w_flappy  = 0.0f;
            w_squashy = 1.0f;
        }
        else{
            float s = 1.0f/(1 + 10*(distance-threshold));
            w_squashy = s;
            w_flappy = 0.4*(1-s);

            if(p.y<-0.18f) {
                w_flappy *= 1-s+5*(p.y+0.18f);
            }

/*            w_flappy = (distance-thet)
                    w_squashy
            w = 5*(n-threshold);
            w_squashy = 0.0f;*///1.0f/(n-threshold+1);
        }
        weight_flappy.push_back(-w_flappy);
        weight_squashy.push_back(w_squashy);
    }

    skinning.rest_pose = sphere.position;
    skinning.rest_pose_normal = sphere.normal;
    skinning.deformed  = sphere;

    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(sphere);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}

void load_diagonal_translate_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);       // no rotation

    joint_geometry g0_0   = {{ 0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g0_1   = {{ 0.25f,0.25f,0.0f},q0};
    joint_geometry g1     = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g2     = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0_0, g1, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0_0},{1,g0_1},{2,g0_0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1},{1,g1},{2,g1}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2}};
    skeleton.anim = {anim_g0, anim_g1, anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(3.14f)* v;

            const vec3 p = {u, r*std::cos(theta), r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    weight_flappy.resize(cylinder.position.size());
    weight_flappy.fill(0.0f);

    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f) {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }

    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed  = cylinder;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}

void load_bending_twisting_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f); // rotation of pi/2 around z-axis
    quaternion q2 = quaternion::axis_angle(normalize(vec3{1,0,0}),3.14f/2.0f); // rotation of pi/2 around x-axis
    quaternion q3 = quaternion::axis_angle(normalize(vec3{1,0,0}),-3.14f/2.0f); // rotation of -pi/2 around x-axis



    joint_geometry g0   = {{0.0f,0.05f,0.0f},q0}; // First joint

    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g1_2 = {{0.5f,0.0f,0.0f},q2}; // Second joint, frame 3
    joint_geometry g1_3 = {{0.5f,0.0f,0.0f},q3}; // Second joint, frame 3

    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0},{3,g0},{4,g0},{5,g0},{6,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_0},{3,g1_2},{4,g1_0},{5,g1_3},{6,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2},{3,g2},{4,g2},{5,g2},{6,g2}};
    skeleton.anim = {anim_g0, anim_g1, anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(3.14f)* v;

            const vec3 p = {u, r*std::cos(theta)*1.5f, r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f) {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }


    weight_flappy.resize(cylinder.position.size()); weight_flappy.fill(0.5f);
    weight_squashy.resize(cylinder.position.size()); weight_squashy.fill(1.0f);



    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed  = cylinder;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 6.0f;
}

data_loaded load_bending_twisting_cylinder_data(vcl::buffer<float>& weight_squashy, GLuint shader)
{
    data_loaded data;
    data.skinning_rig = {};
    data.skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 3.14f / 2.0f); // rotation of pi/2 around z-axis
    quaternion q2 = quaternion::axis_angle(normalize(vec3{ 1,0,0 }), 3.14f / 2.0f); // rotation of pi/2 around z-axis
    quaternion q3 = quaternion::axis_angle(normalize(vec3{ 1,0,0 }), -3.14f / 2.0f); // rotation of pi/2 around z-axis



    joint_geometry g0 = { {0.0f,0.05f,0.0f},q0 }; // First joint

    joint_geometry g1_0 = { {0.5f,0.0f,0.0f},q0 }; // Second joint, frame 0
    joint_geometry g1_1 = { {0.5f,0.0f,0.0f},q1 }; // Second joint, frame 1
    joint_geometry g1_2 = { {0.5f,0.0f,0.0f},q2 }; // Second joint, frame 3
    joint_geometry g1_3 = { {0.5f,0.0f,0.0f},q3 }; // Second joint, frame 3

    joint_geometry g2 = { {0.5f,0.0f,0.0f},q0 }; // Third joint (only used to display bone)

    // Skeleton at rest shape
    data.skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = { {0,g0},{1,g0},{2,g0},{3,g0},{4,g0},{5,g0},{6,g0} };
    std::vector<joint_geometry_time> anim_g1 = { {0,g1_0},{1,g1_1},{2,g1_0},{3,g1_2},{4,g1_0},{5,g1_3},{6,g1_0} };
    std::vector<joint_geometry_time> anim_g2 = { {0,g2},{1,g2},{2,g2},{3,g2},{4,g2},{5,g2},{6,g2} };
    data.skeleton.anim = { anim_g0, anim_g1, anim_g2 };

    // Cylinder shape
    mesh cylinder;
    const size_t N = 50;
    const float r = 0.1f;
    data.skinning_rig.clear();
    for (size_t ku = 0; ku < N; ++ku)
    {
        for (size_t kv = 0; kv < N; ++kv)
        {
            const float u = ku / float(N - 1.0f);
            const float v = kv / float(N);

            const float theta = 2 * float(3.14f) * v;

            const vec3 p = { u, r * std::cos(theta) * 1.5f, r * std::sin(theta) };
            const vec3 n = { 0, std::cos(theta), std::sin(theta) };

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N, N, false, true);


    // Skinning weights
    for (size_t ku = 0; ku < N; ++ku)
    {
        const float u = ku / float(N - 1.0f);
        for (size_t kv = 0; kv < N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if (u < 0.5f) {
                w1 = 0.5f * std::pow(u / 0.5f, alpha);
                w0 = 1 - w1;
            }
            else {
                w0 = 0.5f * std::pow(1 - (u - 0.5f) / 0.5f, alpha);
                w1 = 1 - w0;
            }

            skinning_influence influence_bone_0 = { 0, w0 };
            skinning_influence influence_bone_1 = { 1, w1 };
            data.skinning_rig.push_back({ influence_bone_0, influence_bone_1 });
        }
    }

    data.shape = cylinder;
    //weight_flappy.resize(cylinder.position.size()); weight_flappy.fill(0.5f);
    init_local_buffers(data, data.skeleton.rest_pose.size());
    weight_squashy.resize(cylinder.position.size()); weight_squashy.fill(1.0f);



    //skinning.rest_pose = cylinder.position;
    //skinning.rest_pose_normal = cylinder.normal;
    //skinning.deformed = cylinder;
    data.shape.fill_empty_fields();

    //shape_visual.clear();
    //shape_visual = mesh_drawable(cylinder);
    //shape_visual.shader = shader;
    data.shader = shader;

    //timer = timer_interval();
    //timer.t_max = 6.0f;
    data.anim_time_max = 6.0f;
    return data;
}

void load_bending_linear_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 3.14f / 2.0f); // rotation of pi/2 around z-axis
    //quaternion q2 = quaternion::axis_angle(normalize(vec3{ 1,0,0 }), 3.14f / 2.0f); // rotation of pi/2 around x-axis
    //quaternion q3 = quaternion::axis_angle(normalize(vec3{ 1,0,0 }), -3.14f / 2.0f); // rotation of -pi/2 around x-axis



    joint_geometry g0 = { {0.0f,0.0f,0.0f},q0 }; // First joint

    joint_geometry g1_0 = { {0.5f,0.0f,0.0f},q0 }; // Second joint, frame 0
    joint_geometry g1_1 = { {0.5f,0.0f,0.0f},q1 }; // Second joint, frame 1
    //joint_geometry g1_2 = { {0.5f,0.0f,0.0f},q2 }; // Second joint, frame 3
    //joint_geometry g1_3 = { {0.5f,0.0f,0.0f},q3 }; // Second joint, frame 3

    joint_geometry g2 = { {0.5f,0.0f,0.0f},q0 }; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = { {0,g0},{2,g0},{4,g0} };
    std::vector<joint_geometry_time> anim_g1 = { {0,g1_0},{2,g1_1},{4,g1_0} };
    std::vector<joint_geometry_time> anim_g2 = { {0,g2},{2,g2},{4,g2} };
    skeleton.anim = { anim_g0, anim_g1, anim_g2 };

    // Cylinder shape
    mesh cylinder;
    const size_t N = 50;
    const float r = 0.1f;
    skinning.influence.clear();
    for (size_t ku = 0; ku < N; ++ku)
    {
        for (size_t kv = 0; kv < N; ++kv)
        {
            const float u = ku / float(N - 1.0f);
            const float v = kv / float(N);

            const float theta = 2 * float(3.14f) * v;

            const vec3 p = { u, r * std::cos(theta), r * std::sin(theta) };
            const vec3 n = { 0, std::cos(theta), std::sin(theta) };

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N, N, false, true);


    // Skinning weights
    for (size_t ku = 0; ku < N; ++ku)
    {
        const float u = ku / float(N - 1.0f);
        for (size_t kv = 0; kv < N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if (u < 0.5f) {
                w1 = 0.5f * std::pow(u / 0.5f, alpha);
                w0 = 1 - w1;
            }
            else {
                w0 = 0.5f * std::pow(1 - (u - 0.5f) / 0.5f, alpha);
                w1 = 1 - w0;
            }

            skinning_influence influence_bone_0 = { 0, w0 };
            skinning_influence influence_bone_1 = { 1, w1 };
            skinning.influence.push_back({ influence_bone_0, influence_bone_1 });
        }
    }


    weight_flappy.resize(cylinder.position.size()); weight_flappy.fill(0.5f);
    weight_squashy.resize(cylinder.position.size()); weight_squashy.fill(1.0f);



    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed = cylinder;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 4.0f;
}

void load_bending_sinusoidal_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    // rest position data
    quaternion q0 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 0.0f);       // no rotation
    joint_geometry g0 = { {0.0f,0.0f,0.0f},q0 }; // First joint (no rotation)
    joint_geometry g1_0 = { {0.5f,0.0f,0.0f},q0 }; // Second joint (no rotation)
    joint_geometry g2 = { {0.5f,0.0f,0.0f},q0 }; // Third joint (no rotation)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    const size_t sample_count = 500; // smoothness of sin curve
    const float period = 3.14f * 2; // input angle range [0, 2*pi]
    const float sin_amplitude = 3.14f / 2.0f;  // output angle range [-pi/2, pi/2]
    //const float sin_offset = (3.14 / 4.0f) + 0.05f;   // new ouput angle range [0.05, (pi/2)+0.05]
    const float sin_offset = 0.0f;
    const float t_max = 5.0f;
    const float smallest_angle = period / (float)sample_count;

    std::vector<joint_geometry_time> anim_g0, anim_g1, anim_g2;

    //std::cout << "animation data" << std::endl;
    for (size_t key_frame = 0; key_frame < sample_count; key_frame++)
    {
        float time = (t_max / (float)sample_count) * (float)key_frame;
        quaternion q_i = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), (sin(smallest_angle * key_frame) * sin_amplitude) + sin_offset); // rotation sin(theta) around z-axis

        joint_geometry g1_i = { {0.5f,0.0f,0.0f},q_i }; // Second joint (q rotation)

        anim_g0.push_back({ time, g0 });
        anim_g1.push_back({ time, g1_i });
        anim_g2.push_back({ time, g2});

        //std::cout << "(key, time, angle): (" << key_frame << ", " << time << ", " << sin(smallest_angle * key_frame) << ")" << std::endl;
    }

    // last key frame
    anim_g0.push_back({ t_max, g0 });
    anim_g1.push_back({ t_max, g1_0 });
    anim_g2.push_back({ t_max, g2 });
    
    // Skeleton at rest shape
    //skeleton.rest_pose = { anim_g0[0].geometry, anim_g1[0].geometry, anim_g1[0].geometry };


    skeleton.anim = { anim_g0, anim_g1, anim_g2 };


    // Cylinder shape
    mesh cylinder;
    const size_t N = 50;
    const float r = 0.1f;
    skinning.influence.clear();
    for (size_t ku = 0; ku < N; ++ku)
    {
        for (size_t kv = 0; kv < N; ++kv)
        {
            const float u = ku / float(N - 1.0f);
            const float v = kv / float(N);

            const float theta = 2 * float(3.14f) * v;

            const vec3 p = { u, r * std::cos(theta), r * std::sin(theta) };
            const vec3 n = { 0, std::cos(theta), std::sin(theta) };

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N, N, false, true);


    // Skinning weights
    for (size_t ku = 0; ku < N; ++ku)
    {
        const float u = ku / float(N - 1.0f);
        for (size_t kv = 0; kv < N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if (u < 0.5f) {
                w1 = 0.5f * std::pow(u / 0.5f, alpha);
                w0 = 1 - w1;
            }
            else {
                w0 = 0.5f * std::pow(1 - (u - 0.5f) / 0.5f, alpha);
                w1 = 1 - w0;
            }

            skinning_influence influence_bone_0 = { 0, w0 };
            skinning_influence influence_bone_1 = { 1, w1 };
            skinning.influence.push_back({ influence_bone_0, influence_bone_1 });
        }
    }


    weight_flappy.resize(cylinder.position.size()); weight_flappy.fill(0.5f);
    weight_squashy.resize(cylinder.position.size()); weight_squashy.fill(1.0f);



    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed = cylinder;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = t_max;
}

void load_twisting_sinusoidal_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    // rest position data
    quaternion q0 = quaternion::axis_angle(normalize(vec3{ 1,0,0 }), 0.0f);       // no rotation
    joint_geometry g0 = { {0.0f,0.1f,0.0f},q0 }; // First joint (no rotation)
    joint_geometry g1_0 = { {0.5f,0.0f,0.0f},q0 }; // Second joint (no rotation)
    joint_geometry g2 = { {0.5f,0.0f,0.0f},q0 }; // Third joint (no rotation)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    const size_t sample_count = 500; // smoothness of sin curve
    const float period = 3.14f * 2; // input angle range [0, 2*pi]
    const float max_bend_angle = 3.14f / 2.0f;  // output angle range [-pi/2, pi/2]
    const float max_time = 3.0f;
    const float smallest_angle = period / (float)sample_count;

    std::vector<joint_geometry_time> anim_g0, anim_g1, anim_g2;

    //std::cout << "animation data" << std::endl;
    for (size_t key_frame = 0; key_frame < sample_count; key_frame++)
    {
        float time = (max_time / (float)sample_count) * (float)key_frame;
        quaternion q_i = quaternion::axis_angle(normalize(vec3{ 1,0,0 }), sin(smallest_angle * key_frame) * max_bend_angle); // rotation sin(theta) around x-axis

        joint_geometry g1_i = { {0.5f,0.0f,0.0f},q_i }; // Second joint (q rotation)

        anim_g0.push_back({ time, g0 });
        anim_g1.push_back({ time, g1_i });
        anim_g2.push_back({ time, g2 });

        //std::cout << "(key, time, angle): (" << key_frame << ", " << time << ", " << sin(smallest_angle * key_frame) << ")" << std::endl;
    }

    // last key frame
    anim_g0.push_back({ max_time, g0 });
    anim_g1.push_back({ max_time, g1_0 });
    anim_g2.push_back({ max_time, g2 });

    // Skeleton at rest shape
    //skeleton.rest_pose = { anim_g0[0].geometry, anim_g1[0].geometry, anim_g1[0].geometry };


    skeleton.anim = { anim_g0, anim_g1, anim_g2 };


    // Cylinder shape
    mesh cylinder;
    const size_t N = 50;
    const float r = 0.1f;
    skinning.influence.clear();
    for (size_t ku = 0; ku < N; ++ku)
    {
        for (size_t kv = 0; kv < N; ++kv)
        {
            const float u = ku / float(N - 1.0f);
            const float v = kv / float(N);

            const float theta = 2 * float(3.14f) * v;

            const vec3 p = { u, r * std::cos(theta) * 1.5f, r * std::sin(theta) };
            const vec3 n = { 0, std::cos(theta), std::sin(theta) };

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N, N, false, true);


    // Skinning weights
    for (size_t ku = 0; ku < N; ++ku)
    {
        const float u = ku / float(N - 1.0f);
        for (size_t kv = 0; kv < N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if (u < 0.5f) {
                w1 = 0.5f * std::pow(u / 0.5f, alpha);
                w0 = 1 - w1;
            }
            else {
                w0 = 0.5f * std::pow(1 - (u - 0.5f) / 0.5f, alpha);
                w1 = 1 - w0;
            }

            skinning_influence influence_bone_0 = { 0, w0 };
            skinning_influence influence_bone_1 = { 1, w1 };
            skinning.influence.push_back({ influence_bone_0, influence_bone_1 });
        }
    }


    weight_flappy.resize(cylinder.position.size()); weight_flappy.fill(0.5f);
    weight_squashy.resize(cylinder.position.size()); weight_squashy.fill(1.0f);



    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed = cylinder;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = max_time;
}

void load_translate_sinusoidal_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    // rest position data
    quaternion q0 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 0.0f);       // no rotation
    joint_geometry g0_0 = { {0.0f,0.0f,0.0f},q0 }; // First joint
    joint_geometry g1 = { {0.5f,0.0f,0.0f},q0 }; // Second joint 
    joint_geometry g2 = { {0.5f,0.0f,0.0f},q0 }; // Third joint 

    // Skeleton at rest shape
    skeleton.rest_pose = { g0_0, g1, g2 };

    const size_t sample_count = 500; // smoothness of sin curve
    const float period = 3.14f * 2; // input angle range [0, 2*pi]
    const float max_y = 0.35f;  // output range scale [-0.25, 0.25]
    const float max_time = 3.0f;
    const float dy = period / (float)sample_count; 

    std::vector<joint_geometry_time> anim_g0, anim_g1, anim_g2;

    for (size_t key_frame = 0; key_frame < sample_count; key_frame++)
    {
        float time = (max_time / (float)sample_count) * (float)key_frame;

        joint_geometry g0_i = { { 0.0f, sin(dy * key_frame) * max_y, 0.0f}, q0 }; //root joint translate

        anim_g0.push_back({ time, g0_i });
        anim_g1.push_back({ time, g1 });
        anim_g2.push_back({ time, g2 });

    }

    // last key frame
    anim_g0.push_back({ max_time, g0_0 });
    anim_g1.push_back({ max_time, g1 });
    anim_g2.push_back({ max_time, g2 });

    skeleton.anim = { anim_g0, anim_g1, anim_g2 };


    // Cylinder shape
    mesh cylinder;
    const size_t N = 50;
    const float r = 0.1f;
    skinning.influence.clear();
    for (size_t ku = 0; ku < N; ++ku)
    {
        for (size_t kv = 0; kv < N; ++kv)
        {
            const float u = ku / float(N - 1.0f);
            const float v = kv / float(N);

            const float theta = 2 * float(3.14f) * v;

            const vec3 p = { u, r * std::cos(theta), r * std::sin(theta) };
            const vec3 n = { 0, std::cos(theta), std::sin(theta) };

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N, N, false, true);


    // Skinning weights
    for (size_t ku = 0; ku < N; ++ku)
    {
        const float u = ku / float(N - 1.0f);
        for (size_t kv = 0; kv < N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if (u < 0.5f) {
                w1 = 0.5f * std::pow(u / 0.5f, alpha);
                w0 = 1 - w1;
            }
            else {
                w0 = 0.5f * std::pow(1 - (u - 0.5f) / 0.5f, alpha);
                w1 = 1 - w0;
            }

            skinning_influence influence_bone_0 = { 0, w0 };
            skinning_influence influence_bone_1 = { 1, w1 };
            skinning.influence.push_back({ influence_bone_0, influence_bone_1 });
        }
    }


    weight_flappy.resize(cylinder.position.size()); weight_flappy.fill(0.5f);
    weight_squashy.resize(cylinder.position.size()); weight_squashy.fill(1.0f);



    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed = cylinder;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = max_time;
}

void load_bending_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f*0.8);
    quaternion q2 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f); // rotation of pi/2 around z-axis


    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g1_2 = {{0.5f,0.0f,0.0f},q2}; // Second joint, frame 2
    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0},{3,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_2},{3,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2},{3,g2}};
    skeleton.anim = {anim_g0, anim_g1, anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(3.14f)* v;

            const vec3 p = {u, r*std::cos(theta), r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f) {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }


    weight_flappy.resize(cylinder.position.size()); weight_flappy.fill(0.5f);
    weight_squashy.resize(cylinder.position.size()); weight_squashy.fill(1.0f);
    //init_local_buffers(squashing_power_buffer, cylinder.position.size());


    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed  = cylinder;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 3.0f;
}

data_loaded load_bending_cylinder_data(vcl::buffer<float>& weight_squashy, GLuint shader)
{
    data_loaded data;
    data.skinning_rig = {};
    data.skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 3.14f / 2.0f * 0.8);
    quaternion q2 = quaternion::axis_angle(normalize(vec3{ 0,0,1 }), 3.14f / 2.0f); // rotation of pi/2 around z-axis


    joint_geometry g0 = { {0.0f,0.0f,0.0f},q0 }; // First joint
    joint_geometry g1_0 = { {0.5f,0.0f,0.0f},q0 }; // Second joint, frame 0
    joint_geometry g1_1 = { {0.5f,0.0f,0.0f},q1 }; // Second joint, frame 1
    joint_geometry g1_2 = { {0.5f,0.0f,0.0f},q2 }; // Second joint, frame 2
    joint_geometry g2 = { {0.5f,0.0f,0.0f},q0 }; // Third joint (only used to display bone)

    // Skeleton at rest shape
    data.skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = { {0,g0},{1,g0},{2,g0},{3,g0} };
    std::vector<joint_geometry_time> anim_g1 = { {0,g1_0},{1,g1_1},{2,g1_2},{3,g1_0} };
    std::vector<joint_geometry_time> anim_g2 = { {0,g2},{1,g2},{2,g2},{3,g2} };
    data.skeleton.anim = { anim_g0, anim_g1, anim_g2 };

    // Cylinder shape
    mesh cylinder;
    const size_t N = 50;
    const float r = 0.1f;
    data.skinning_rig.clear();
    for (size_t ku = 0; ku < N; ++ku)
    {
        for (size_t kv = 0; kv < N; ++kv)
        {
            const float u = ku / float(N - 1.0f);
            const float v = kv / float(N);

            const float theta = 2 * float(3.14f) * v;

            const vec3 p = { u, r * std::cos(theta), r * std::sin(theta) };
            const vec3 n = { 0, std::cos(theta), std::sin(theta) };

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N, N, false, true);


    // Skinning weights
    for (size_t ku = 0; ku < N; ++ku)
    {
        const float u = ku / float(N - 1.0f);
        for (size_t kv = 0; kv < N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if (u < 0.5f) {
                w1 = 0.5f * std::pow(u / 0.5f, alpha);
                w0 = 1 - w1;
            }
            else {
                w0 = 0.5f * std::pow(1 - (u - 0.5f) / 0.5f, alpha);
                w1 = 1 - w0;
            }

            skinning_influence influence_bone_0 = { 0, w0 };
            skinning_influence influence_bone_1 = { 1, w1 };
            data.skinning_rig.push_back({ influence_bone_0, influence_bone_1 });
        }
    }


    data.shape = cylinder;
    init_local_buffers(data, data.skeleton.rest_pose.size());
    /*data.weight_flappy.resize(cylinder.position.size());
    data.weight_flappy.fill(0.5f);
    init_local_buffers(data.squashing_power_buffer, cylinder.position.size());*/
    weight_squashy.resize(cylinder.position.size());
    weight_squashy.fill(1.0f);


    /*skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed = cylinder;
    skinning.deformed.fill_empty_fields();*/
    data.shape.fill_empty_fields();

    data.shader = shader;
    data.anim_time_max = 3.0f;
    return data;
}

void load_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f); // rotation of pi/2 around z-axis


    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2}};
    skeleton.anim = {anim_g0,anim_g1,anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(3.14f)* v;

            const vec3 p = {u, r*std::cos(theta), r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f) {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }

    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed  = cylinder;
    skinning.deformed.fill_empty_fields();


    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;

}


void load_rectangle_data(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);      // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{1,0,0}),3.14f); // rotation of pi around x-axis
    quaternion q2 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0); // rotation of pi around x-axis

    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g1_2 = {{0.5f,0.0f,0.0f},q2}; // Second joint, frame 1
    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0},{3,g0},{4,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_0},{3,g1_2},{4,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2},{3,g2},{4,g2}};
    skeleton.anim = {anim_g0,anim_g1,anim_g2};

    // Cylinder shape
    float const r = 0.1f;
    mesh shape;
    shape = mesh_primitive_bar_grid(50,10, 10,{0,-r,-r},{1,0,0},{0,2*r,0},{0,0,2*r});
    skinning.influence.clear();

    // Skinning weights
    size_t const N = shape.position.size();
    for(size_t k=0; k<N; ++k) {
        float const u = shape.position[k].x;

        float w0, w1;
        const float alpha = 3.0f; // power for skinning weights evolution
        if(u<0.5f) {
            w1 = 0.5f*std::pow(u/0.5f, alpha);
            w0 = 1-w1;
        }
        else {
            w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
            w1 = 1-w0;
        }

        skinning_influence influence_bone_0 = {0, w0};
        skinning_influence influence_bone_1 = {1, w1};
        skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
    }

    skinning.rest_pose = shape.position;
    skinning.rest_pose_normal = shape.normal;
    skinning.deformed  = shape;
    skinning.deformed.fill_empty_fields();

    shape_visual.clear();
    shape_visual = mesh_drawable(shape);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}



void load_character_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader, int& real_texture_id)
{
    skinning.influence = {};
    timer = timer_interval();

    const float scaling = 0.005f;

    // Load mesh
    buffer<buffer<int> > vertex_correspondance;
    mesh character = mesh_load_file_obj("assets/marine/mesh.obj", vertex_correspondance);

    skeleton.connectivity   = read_skeleton_connectivity("assets/marine/skeleton_connectivity");
    skeleton.rest_pose      = read_skeleton_geometry("assets/marine/skeleton_geometry_local", scaling);
    buffer<buffer<skinning_influence>> influence = read_skinning_influence("assets/marine/skinning_data");
    skinning.influence = map_correspondance(influence, vertex_correspondance);

    // 3 possibles animations:
    // skeleton.anim           = read_skeleton_animation("scenes/sources/skinning/assets/marine/skeleton_animation_idle", scaling); // t_max = 4.0f
    // skeleton.anim           = read_skeleton_animation("scenes/sources/skinning/assets/marine/skeleton_animation_walk", scaling); // t_max = 1.0f;
    skeleton.anim           = read_skeleton_animation("assets/marine/skeleton_animation_run", scaling);  // t_max = 0.733f;
    timer.t_max = 0.733f;
    
    GLuint texture_id = create_texture_gpu(image_load_png("assets/marine/texture.png"));

    for(vec3& p : character.position) p *= scaling; // scale vertices of the mesh
    shape_visual.clear();
    shape_visual = mesh_drawable(character);
    shape_visual.shader = shader;
    shape_visual.uniform.shading.specular = 0.1f;
    shape_visual.uniform.shading.specular_exponent = 8;

    character.fill_empty_fields();
    skinning.rest_pose = character.position;
    skinning.rest_pose_normal = character.normal;
    skinning.deformed  = character;
    skinning.deformed.fill_empty_fields();

    weight_flappy.resize(character.position.size());
    weight_flappy.fill(1.0f);

    shape_visual.texture_id = texture_id;
    real_texture_id = texture_id;
}




data_loaded load_basic_data_directory( parameters_basic_loader const& param)
{
    std::string const& dir = param.common_dir;
    GLuint shader = param.shader;

    data_loaded data;

    buffer<buffer<int> > vertex_correspondance;
    data.shape = mesh_load_file_obj(dir+"mesh.obj", vertex_correspondance);
    data.shape.position *= param.scaling;
    data.shape.normal = normal(data.shape.position, data.shape.connectivity);
    data.shape.fill_empty_fields();


    buffer<std::string> joint_name;      read_from_file(dir+"skeleton_joint_name", joint_name);
    buffer<int> joint_parent;            read_from_file(dir+"skeleton_connectivity_parent", joint_parent);
    buffer<vec3> joint_translation;      read_from_file(dir+"skeleton_rest_pose_translation", joint_translation);
    buffer<quaternion> joint_quaternion; read_from_file(dir+"skeleton_rest_pose_rotation", joint_quaternion);
    buffer<buffer<int>> rig_joint;       read_from_file(dir+"rig_joint", rig_joint);
    buffer<buffer<float>> rig_weights;   read_from_file(dir+"rig_weights", rig_weights);

    data.texture_id = create_texture_gpu(image_load_png(dir+"texture.png"));


    if(check_file_exist(dir+"skeleton_symmetry"))
    {
        buffer<int> symmetry;
        read_from_file(dir+"skeleton_symmetry", symmetry);
        size_t N = symmetry.size()/2;
        for(size_t k=0; k<N; ++k){
            data.symmetry[ symmetry[2*k] ] = symmetry[2*k+1];
            data.symmetry[ symmetry[2*k+1] ] = symmetry[2*k];
        }
    }



    int N_joint = joint_name.size();
    assert_vcl_no_msg(N_joint>0);
    for(int k=0; k<N_joint; ++k)
    {
        data.skeleton.rest_pose.push_back( {param.scaling*joint_translation[k], joint_quaternion[k]} );
        data.skeleton.connectivity.push_back( {joint_parent[k], joint_name[k]} );
    }
    data.skeleton.anim.resize(N_joint);
    for(int k=0; k<N_joint; ++k)
    {
        data.skeleton.anim[k].resize(3);
        data.skeleton.anim[k][0].time = 0;
        data.skeleton.anim[k][1].time = 1;
        data.skeleton.anim[k][2].time = 2;

        data.skeleton.anim[k][0].geometry = data.skeleton.rest_pose[k];
        data.skeleton.anim[k][1].geometry = data.skeleton.rest_pose[k];
        data.skeleton.anim[k][2].geometry = data.skeleton.rest_pose[k];
    }

//    data.skeleton.anim[9][0].geometry.r = quaternion::axis_angle({1,0,0},-3.14f/4.0f);
//    data.skeleton.anim[9][1].geometry.r = quaternion::axis_angle({1,0,0},+3.14f/4.0f);
//    data.skeleton.anim[9][2].geometry.r = data.skeleton.anim[9][0].geometry.r;


    data.anim_time_max = 2.0f;


    assert_vcl_no_msg(rig_joint.size() == rig_weights.size());
    data.skinning_rig.resize(rig_joint.size());
    for(int k=0; k< int(rig_joint.size()); ++k)
    {
        assert_vcl_no_msg( rig_joint[k].size() == rig_weights[k].size() );
        int N_dependencies = rig_joint[k].size();
        data.skinning_rig[k].resize(N_dependencies);
        for(int kb=0; kb<N_dependencies; ++kb)
        {
            data.skinning_rig[k][kb].joint = rig_joint[k][kb];
            data.skinning_rig[k][kb].weight = rig_weights[k][kb];
        }
    }

    data.skinning_rig = map_correspondance(data.skinning_rig, vertex_correspondance);


    int N_vertex = data.shape.position.size();
    data.weight_flappy.resize(N_vertex);
    data.weight_flappy.fill(0.5f);

    data.shader = shader;

    assert_vcl_no_msg(check_integrity(data));
    return data;
}


data_loaded load_basic_animation_data_directory(parameters_basic_loader const& param)
{
    std::string const& dir = param.common_dir;
    GLuint shader = param.shader;

    data_loaded data;

    buffer<buffer<int> > vertex_correspondance;
    data.shape = mesh_load_file_obj(dir + "mesh.obj", vertex_correspondance);
    data.shape.position *= param.scaling;
    data.shape.normal = normal(data.shape.position, data.shape.connectivity);
    data.shape.fill_empty_fields();


    buffer<std::string> joint_name;      read_from_file(dir + "skeleton_joint_name", joint_name);
    buffer<int> joint_parent;            read_from_file(dir + "skeleton_connectivity_parent", joint_parent);
    buffer<vec3> joint_translation;      read_from_file(dir + "skeleton_rest_pose_translation", joint_translation);
    buffer<quaternion> joint_quaternion; read_from_file(dir + "skeleton_rest_pose_rotation", joint_quaternion);
    //buffer<joint_connectivity> connectivity = read_skeleton_connectivity("assets/flower/skeleton_connectivity");
    //buffer<joint_geometry> rest_pose = read_skeleton_geometry("assets/flower/skeleton_geometry_local", param.scaling);
    buffer<buffer<int>> rig_joint;       read_from_file(dir + "rig_joint", rig_joint);
    buffer<buffer<float>> rig_weights;   read_from_file(dir + "rig_weights", rig_weights);

    data.texture_id = create_texture_gpu(image_load_png(dir + "texture.png"));


    if (check_file_exist(dir + "skeleton_symmetry"))
    {
        buffer<int> symmetry;
        read_from_file(dir + "skeleton_symmetry", symmetry);
        size_t N = symmetry.size() / 2;
        for (size_t k = 0; k < N; ++k) {
            data.symmetry[symmetry[2 * k]] = symmetry[2 * k + 1];
            data.symmetry[symmetry[2 * k + 1]] = symmetry[2 * k];
        }
    }



    int N_joint = joint_name.size();
    assert_vcl_no_msg(N_joint > 0);
    for (int k = 0; k < N_joint; ++k)
    {
        data.skeleton.rest_pose.push_back({ param.scaling * joint_translation[k], joint_quaternion[k] });
        data.skeleton.connectivity.push_back({ joint_parent[k], joint_name[k] });
    }
    data.skeleton.anim.resize(N_joint);
    /*for (int k = 0; k < N_joint; ++k)
    {
        data.skeleton.anim[k].resize(3);
        data.skeleton.anim[k][0].time = 0;
        data.skeleton.anim[k][1].time = 1;
        data.skeleton.anim[k][2].time = 2;

        data.skeleton.anim[k][0].geometry = data.skeleton.rest_pose[k];
        data.skeleton.anim[k][1].geometry = data.skeleton.rest_pose[k];
        data.skeleton.anim[k][2].geometry = data.skeleton.rest_pose[k];
    }*/

    //    data.skeleton.anim[9][0].geometry.r = quaternion::axis_angle({1,0,0},-3.14f/4.0f);
    //    data.skeleton.anim[9][1].geometry.r = quaternion::axis_angle({1,0,0},+3.14f/4.0f);
    //    data.skeleton.anim[9][2].geometry.r = data.skeleton.anim[9][0].geometry.r;


    //data.anim_time_max = 2.0f;
    if (check_file_exist(dir + "skeleton_animation"))
    {
        data.skeleton.anim = read_skeleton_animation(dir + "skeleton_animation", param.scaling);  // t_max = 0.733f;
        data.anim_time_max = find_animation_length(data.skeleton.anim);
    }
    else
    {
        data.skeleton.anim.resize(N_joint);
        for (int k = 0; k < N_joint; ++k)
        {
            data.skeleton.anim[k].resize(3);
            data.skeleton.anim[k][0].time = 0;
            data.skeleton.anim[k][1].time = 1;
            data.skeleton.anim[k][2].time = 2;

            data.skeleton.anim[k][0].geometry = data.skeleton.rest_pose[k];
            data.skeleton.anim[k][1].geometry = data.skeleton.rest_pose[k];
            data.skeleton.anim[k][2].geometry = data.skeleton.rest_pose[k];
        }
        data.anim_time_max = 2.0f;
    }
    //timer.t_max = 0.733f;


    assert_vcl_no_msg(rig_joint.size() == rig_weights.size());
    data.skinning_rig.resize(rig_joint.size());
    for (int k = 0; k< int(rig_joint.size()); ++k)
    {
        assert_vcl_no_msg(rig_joint[k].size() == rig_weights[k].size());
        int N_dependencies = rig_joint[k].size();
        data.skinning_rig[k].resize(N_dependencies);
        for (int kb = 0; kb < N_dependencies; ++kb)
        {
            data.skinning_rig[k][kb].joint = rig_joint[k][kb];
            data.skinning_rig[k][kb].weight = rig_weights[k][kb];
        }
    }

    data.skinning_rig = map_correspondance(data.skinning_rig, vertex_correspondance);


   /* int N_vertex = data.shape.position.size();
    data.weight_flappy.resize(N_vertex);
    data.weight_flappy.fill(0.5f);
    data.squashing_power_buffer.resize(N_vertex);
    data.squashing_power_buffer.fill(1.0);*/

    init_local_buffers(data, N_joint);
    data.shader = shader;

    assert_vcl_no_msg(check_integrity(data));
    return data;
}

data_loaded load_basic_animation_data_directory_withProxyGeo(parameters_basic_loader const& param)
{
    std::string const& dir = param.common_dir;
    GLuint shader = param.shader;

    data_loaded data;

    buffer<buffer<int> > vertex_correspondance;
    data.shape = mesh_load_file_obj(dir + "proxy.obj", vertex_correspondance);
    data.shape.position *= param.scaling;
    data.shape.normal = normal(data.shape.position, data.shape.connectivity);
    data.shape.fill_empty_fields();
    
    data.proxy_shape = mesh_load_file_obj(dir + "mesh.obj", vertex_correspondance);
    data.proxy_shape.position *= param.scaling;
    data.proxy_shape.normal = normal(data.proxy_shape.position, data.proxy_shape.connectivity);
    data.proxy_shape.fill_empty_fields();
    data.deform_proxy = true;


    buffer<std::string> joint_name;      read_from_file(dir + "skeleton_joint_name", joint_name);
    buffer<int> joint_parent;            read_from_file(dir + "skeleton_connectivity_parent", joint_parent);
    buffer<vec3> joint_translation;      read_from_file(dir + "skeleton_rest_pose_translation", joint_translation);
    buffer<quaternion> joint_quaternion; read_from_file(dir + "skeleton_rest_pose_rotation", joint_quaternion);
    //buffer<joint_connectivity> connectivity = read_skeleton_connectivity("assets/flower/skeleton_connectivity");
    //buffer<joint_geometry> rest_pose = read_skeleton_geometry("assets/flower/skeleton_geometry_local", param.scaling);
    buffer<buffer<int>> rig_joint;       read_from_file(dir + "rig_joint", rig_joint);
    buffer<buffer<float>> rig_weights;   read_from_file(dir + "rig_weights", rig_weights);

    data.texture_id = create_texture_gpu(image_load_png(dir + "texture.png"));


    if (check_file_exist(dir + "skeleton_symmetry"))
    {
        buffer<int> symmetry;
        read_from_file(dir + "skeleton_symmetry", symmetry);
        size_t N = symmetry.size() / 2;
        for (size_t k = 0; k < N; ++k) {
            data.symmetry[symmetry[2 * k]] = symmetry[2 * k + 1];
            data.symmetry[symmetry[2 * k + 1]] = symmetry[2 * k];
        }
    }



    int N_joint = joint_name.size();
    assert_vcl_no_msg(N_joint > 0);
    for (int k = 0; k < N_joint; ++k)
    {
        data.skeleton.rest_pose.push_back({ param.scaling * joint_translation[k], joint_quaternion[k] });
        data.skeleton.connectivity.push_back({ joint_parent[k], joint_name[k] });
    }
    data.skeleton.anim.resize(N_joint);
    /*for (int k = 0; k < N_joint; ++k)
    {
        data.skeleton.anim[k].resize(3);
        data.skeleton.anim[k][0].time = 0;
        data.skeleton.anim[k][1].time = 1;
        data.skeleton.anim[k][2].time = 2;

        data.skeleton.anim[k][0].geometry = data.skeleton.rest_pose[k];
        data.skeleton.anim[k][1].geometry = data.skeleton.rest_pose[k];
        data.skeleton.anim[k][2].geometry = data.skeleton.rest_pose[k];
    }*/

    //    data.skeleton.anim[9][0].geometry.r = quaternion::axis_angle({1,0,0},-3.14f/4.0f);
    //    data.skeleton.anim[9][1].geometry.r = quaternion::axis_angle({1,0,0},+3.14f/4.0f);
    //    data.skeleton.anim[9][2].geometry.r = data.skeleton.anim[9][0].geometry.r;


    //data.anim_time_max = 2.0f;
    if (check_file_exist(dir + "skeleton_animation"))
    {
        data.skeleton.anim = read_skeleton_animation(dir + "skeleton_animation", param.scaling);  // t_max = 0.733f;
        data.anim_time_max = find_animation_length(data.skeleton.anim);
    }
    else
    {
        data.skeleton.anim.resize(N_joint);
        for (int k = 0; k < N_joint; ++k)
        {
            data.skeleton.anim[k].resize(3);
            data.skeleton.anim[k][0].time = 0;
            data.skeleton.anim[k][1].time = 1;
            data.skeleton.anim[k][2].time = 2;

            data.skeleton.anim[k][0].geometry = data.skeleton.rest_pose[k];
            data.skeleton.anim[k][1].geometry = data.skeleton.rest_pose[k];
            data.skeleton.anim[k][2].geometry = data.skeleton.rest_pose[k];
        }
        data.anim_time_max = 2.0f;
    }
    //timer.t_max = 0.733f;


    assert_vcl_no_msg(rig_joint.size() == rig_weights.size());
    data.skinning_rig.resize(rig_joint.size());
    for (int k = 0; k< int(rig_joint.size()); ++k)
    {
        assert_vcl_no_msg(rig_joint[k].size() == rig_weights[k].size());
        int N_dependencies = rig_joint[k].size();
        data.skinning_rig[k].resize(N_dependencies);
        for (int kb = 0; kb < N_dependencies; ++kb)
        {
            data.skinning_rig[k][kb].joint = rig_joint[k][kb];
            data.skinning_rig[k][kb].weight = rig_weights[k][kb];
        }
    }

    data.skinning_rig = map_correspondance(data.skinning_rig, vertex_correspondance);


    /* int N_vertex = data.shape.position.size();
     data.weight_flappy.resize(N_vertex);
     data.weight_flappy.fill(0.5f);
     data.squashing_power_buffer.resize(N_vertex);
     data.squashing_power_buffer.fill(1.0);*/

    init_local_buffers(data, N_joint);
    data.shader = shader;

    assert_vcl_no_msg(check_integrity(data));
    return data;
}

void init_local_buffers(data_loaded &data, unsigned int N_joint)
{
    int N_vertex = data.shape.position.size();
    data.weight_flappy.resize(N_vertex);
    data.weight_flappy.fill(0.5f);
    data.squashing_power_buffer.resize(N_joint);
    data.squashing_power_buffer.fill(1.0);
    data.flapping_power_translate_buffer.resize(N_joint);
    data.flapping_power_translate_buffer.fill(1.0);
    data.flapping_power_unified_buffer.resize(N_joint);
    data.flapping_power_unified_buffer.fill(1.0);
    data.bendy_stretch_power_buffer.resize(N_joint);
    data.bendy_stretch_power_buffer.fill(0.0);
    data.follow_through_power_translate_buffer.resize(N_joint);
    data.follow_through_power_translate_buffer.fill(0.5);
    data.follow_through_power_rotation_buffer.resize(N_joint);
    data.follow_through_power_rotation_buffer.fill(0.5);
    data.flapping_power_twist_buffer.resize(N_joint);
    data.flapping_power_twist_buffer.fill(1.0);
    data.flapping_power_bend_buffer.resize(N_joint);
    data.flapping_power_bend_buffer.fill(1.0);

}

void init_local_buffers(vcl::buffer<float>& squashing_power_buffer, vcl::buffer<float>& flapping_power_translate_buffer, vcl::buffer<float>& flapping_power_unified_buffer, vcl::buffer<float>& bendy_stretch_power_buffer, vcl::buffer<float>& follow_through_power_translate_buffer, vcl::buffer<float>& follow_through_power_rotation_buffer, vcl::buffer<float>& flapping_power_twist_buffer, vcl::buffer<float>& flapping_power_bend_buffer, unsigned int N_joint)
{
    squashing_power_buffer.resize(N_joint);
    squashing_power_buffer.fill(1.0);
    flapping_power_translate_buffer.resize(N_joint);
    flapping_power_translate_buffer.fill(1.0);
    flapping_power_unified_buffer.resize(N_joint);
    flapping_power_unified_buffer.fill(1.0);
    bendy_stretch_power_buffer.resize(N_joint);
    bendy_stretch_power_buffer.fill(0.0);
    follow_through_power_translate_buffer.resize(N_joint);
    follow_through_power_translate_buffer.fill(0.5);
    follow_through_power_rotation_buffer.resize(N_joint);
    follow_through_power_rotation_buffer.fill(0.5);
    flapping_power_twist_buffer.resize(N_joint);
    flapping_power_twist_buffer.fill(1.0);
    flapping_power_bend_buffer.resize(N_joint);
    flapping_power_bend_buffer.fill(1.0);
}
 data_loaded load_girafe2_data(GLuint shader)
{
    //data_loaded data = load_basic_data_directory({ "assets/girafe/", shader, 2.0f });
    data_loaded data = load_basic_animation_data_directory( {"assets/girafe/", shader, .02f} );
    data.shape.inverse_triangle_orientation();
    data.shape.normal = normal(data.shape.position, data.shape.connectivity);
    return data;
}

data_loaded load_spot_data(GLuint shader)
{
    // data_loaded data = load_basic_data_directory({ "assets/spot/", shader });
    data_loaded data = load_basic_animation_data_directory( {"assets/spot/", shader} );

    return data;
}

data_loaded load_flower_data(GLuint shader)
{
    //data_loaded data = load_basic_data_directory({ "assets/flower/", shader });
    data_loaded data = load_basic_animation_data_directory({ "assets/flower/", shader, 0.25f });

    return data;
}

//data_loaded load_doll_data(GLuint shader)
//{
//    //data_loaded data = load_basic_data_directory({ "assets/flower/", shader });
//    data_loaded data = load_basic_animation_data_directory_withProxyGeo({ "assets/doll/", shader, 0.25f });
//
//    return data;
//}
data_loaded load_doll_data(GLuint shader)
{
    //bool animation_exists = check_file_exist("assets/custom/skeleton_animation");
    //data_loaded data = animation_exists ? load_basic_animation_data_directory({ "assets/custom/", shader, 0.5f }) : load_basic_data_directory({ "assets/custom/", shader, 0.5f });

    data_loaded data = load_basic_animation_data_directory_withProxyGeo({ "assets/doll/", shader });

    return data;
}

data_loaded load_dragon_data(GLuint shader)
{
    //data_loaded data = load_basic_data_directory({ "assets/dragon/", shader });
    data_loaded data = load_basic_animation_data_directory( {"assets/dragon/", shader} );
    return data;
}

data_loaded load_snail_data(GLuint shader)
{
    //data_loaded data = load_basic_data_directory({ "assets/snail/", shader });
    data_loaded data = load_basic_animation_data_directory( {"assets/snail/", shader} );
    return data;
}
data_loaded load_custom_data(GLuint shader)
{
    //bool animation_exists = check_file_exist("assets/custom/skeleton_animation");
    //data_loaded data = animation_exists ? load_basic_animation_data_directory({ "assets/custom/", shader, 0.5f }) : load_basic_data_directory({ "assets/custom/", shader, 0.5f });

    data_loaded data = load_basic_animation_data_directory({ "assets/custom/", shader });

    return data;
}




bool check_integrity(data_loaded const& data)
{
    size_t const N_vertex = data.shape.position.size() || data.proxy_shape.position.size();
    check_vcl_no_msg(N_vertex>0);

    size_t const N_joint = data.skeleton.rest_pose.size();
    check_vcl_no_msg(N_joint>0);

    check_vcl_no_msg(data.skeleton.connectivity.size()==N_joint);


    return true;
}


void load_girafe_data(skeleton_structure& skeleton,
                      skinning_structure& skinning,
                      vcl::buffer<float>& weight_flappy,
                      mesh_drawable& shape_visual,
                      timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    timer = timer_interval();

    const float scaling = 0.02f;

    // Load mesh
    buffer<buffer<int> > vertex_correspondance;
    mesh character = mesh_load_file_obj("scenes/sources/squashy_skinning/assets/girafe/girafe.obj", vertex_correspondance);
    character.position *= scaling;


    skeleton.connectivity   = read_skeleton_connectivity("scenes/sources/squashy_skinning/assets/girafe/skeleton_connectivity");
    skeleton.rest_pose      = read_skeleton_geometry("scenes/sources/squashy_skinning/assets/girafe/skeleton_geometry_local", scaling);
    buffer<buffer<skinning_influence>> influence = read_skinning_influence_variable("scenes/sources/squashy_skinning/assets/girafe/skinning_data");

    skinning.influence = map_correspondance(influence, vertex_correspondance);



    skeleton.anim.resize(skeleton.rest_pose.size());
    for(int k=0; k<int(skeleton.rest_pose.size()); ++k)
    {
        skeleton.anim[k].resize(2);
        skeleton.anim[k][0].time = 0.0f;
        skeleton.anim[k][0].geometry = skeleton.rest_pose[k];
        skeleton.anim[k][1].time = 1.0f;
        skeleton.anim[k][1].geometry = skeleton.rest_pose[k];
    }

    // 3 possibles animations:
    // skeleton.anim           = read_skeleton_animation("scenes/sources/squashy_skinning/assets/marine/skeleton_animation_idle", scaling); // t_max = 4.0f
    // skeleton.anim           = read_skeleton_animation("scenes/sources/squashy_skinning/assets/marine/skeleton_animation_walk", scaling); // t_max = 1.0f;
    //skeleton.anim           = read_skeleton_animation("scenes/sources/squashy_skinning/assets/marine/skeleton_animation_run", scaling);  // t_max = 0.733f;

    timer.t_max = 1.0f;

    //GLuint texture_id = create_texture_gpu(image_load_png("scenes/sources/squashy_skinning/assets/marine/texture.png"));

    //for(vec3& p : character.position) p *= scaling; // scale vertices of the mesh
    shape_visual.clear();
    shape_visual = mesh_drawable(character);
    shape_visual.shader = shader;

    character.fill_empty_fields();
    skinning.rest_pose = character.position;
    skinning.rest_pose_normal = character.normal;
    skinning.deformed  = character;
    skinning.deformed.fill_empty_fields();

    weight_flappy.resize(character.position.size());
    weight_flappy.fill(1.0f);


}




buffer<buffer<skinning_influence> > read_skinning_influence_variable(const std::string& filename)
{
    buffer<buffer<skinning_influence> > influence;

    std::ifstream fid(filename);

//    // first line = number of influence per pertex (fixed for all vertices)
//    size_t N_bone_influence=0;
//    fid >> N_bone_influence;

//    assert_vcl(fid.good(), "Cannot read file "+filename);
//    assert_vcl_no_msg(N_bone_influence>0 && N_bone_influence<=6);

    // Read influence associated to each vertex



    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        std::istringstream sline(line);
        std::vector<std::string> vs;
        std::string temp;
        while( std::getline(sline,temp,' ') ) {
            vs.push_back(temp);
        }

        int const N = vs.size()/2;
        buffer<skinning_influence> skinning_vertex;
        for(int k=0; k<N; ++k)
        {
            skinning_influence s;
            s.joint = std::stoi(vs[2*k]);
            s.weight = std::stof(vs[2*k+1]);
            skinning_vertex.push_back(s);
        }


//        // read list of [bone index] [skinning weights]
//        for(size_t k=0; k<N_bone_influence && fid.good(); ++k)
//            fid >> skinning_vertex[k].joint >> skinning_vertex[k].weight;

        if(fid.good())
            influence.push_back(skinning_vertex);
    }

    fid.close();


    // normalize skinning weights (sum up to one)
    for(size_t kv=0; kv<influence.size(); ++kv)
    {
        float w = 0.0f;
        // compute weight sum
        for(size_t k=0; k<influence[kv].size(); ++k) {
            w += influence[kv][k].weight;
        }
        // normalize
        if(w>1e-5f)
            for(size_t k=0; k<influence[kv].size(); ++k)
                influence[kv][k].weight /= w;



    }

    return influence;

}

buffer<buffer<skinning_influence> > read_skinning_influence(const std::string& filename)
{
    buffer<buffer<skinning_influence> > influence;

    std::ifstream fid(filename);

    // first line = number of influence per pertex (fixed for all vertices)
    size_t N_bone_influence=0;
    fid >> N_bone_influence;

    assert_vcl(fid.good(), "Cannot read file "+filename);
    assert_vcl_no_msg(N_bone_influence>0 && N_bone_influence<=6);

    // Read influence associated to each vertex
    std::vector<skinning_influence> skinning_vertex;
    skinning_vertex.resize(N_bone_influence);

    while(fid.good())
    {
        // read list of [bone index] [skinning weights]
        for(size_t k=0; k<N_bone_influence && fid.good(); ++k)
            fid >> skinning_vertex[k].joint >> skinning_vertex[k].weight;

        if(fid.good())
            influence.push_back(skinning_vertex);
    }

    fid.close();


    // normalize skinning weights (sum up to one)
    for(size_t kv=0; kv<influence.size(); ++kv)
    {
        float w = 0.0f;
        // compute weight sum
        for(size_t k=0; k<N_bone_influence; ++k)
            w += influence[kv][k].weight;
        // normalize
        if(w>1e-5f)
            for(size_t k=0; k<N_bone_influence; ++k)
                influence[kv][k].weight /= w;

    }

    return influence;
}

float find_animation_length(const buffer<buffer<joint_geometry_time>>& anim)
{
    float end_time = 0;
    float prev_end_time = 0;
    for (auto joint : anim)
    {
        for (auto key : joint)
        {
            if (key.time > end_time)
            {
                prev_end_time = end_time;
                end_time = key.time;
            }
        }
    }
    // stop timer just before final keyframe to allow proper interpolation
    return end_time - (end_time - prev_end_time) / 10;
}

buffer<buffer<joint_geometry_time> > read_skeleton_animation(const std::string& filename, float scaling)
{
    buffer<buffer<joint_geometry_time> > skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        size_t N_key=0;
        fid >> N_key;

        if(fid.good())
        {
            buffer<joint_geometry_time> animated_joint;
            animated_joint.resize(N_key);

            for(size_t k_key=0; k_key<N_key; ++k_key)
            {
                float key_time;
                vec3 p;
                vec4 q;

                fid >> key_time;
                fid >> p.x >> p.y >> p.z;
                fid >> q.x >> q.y >> q.z >> q.w;

                q = normalize(q);

                animated_joint[k_key] = {key_time, {p*scaling,q} };
            }

            skeleton.push_back(animated_joint);
        }
    }

    fid.close();

    return skeleton;
}

buffer<joint_connectivity> read_skeleton_connectivity(const std::string& filename)
{
    buffer<joint_connectivity> skeleton;

    std::ifstream fid(filename);

    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            int k;
            int parent;
            std::string name;

            sstream >> k >> parent >> name;

            skeleton.push_back({parent,name});
        }
    }

    fid.close();

    return skeleton;
}

buffer<joint_geometry> read_skeleton_geometry(const std::string& filename, float scaling)
{
    buffer<joint_geometry> skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            vec3 p;
            quaternion q;

            sstream >> p.x >> p.y >> p.z;
            sstream >> q.x >> q.y >> q.z >> q.w;

            //q = normalize(q);

            skeleton.push_back({p*scaling,q});
        }
    }

    fid.close();

    return skeleton;
}


#endif
