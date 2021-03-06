#pragma once

#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "helper_skinning.hpp"
#include "velocity_tracker.hpp"
#include "helper_velocity_skinning.hpp"
#include "vcl_advanced/vcl_advanced.hpp"

#include <set>

namespace vcl{
struct timer_tic_toc : timer_basic
{
public:
    using timer_basic::timer_basic;

    size_t counter=0;
    float average_timing=0.0f;

    void tic()
    {
        if(!timer_basic::running)
            timer_basic::start();
    }
    void toc()
    {
        if(timer_basic::running) {
            timer_basic::update();
            timer_basic::stop();

            ++counter;
            average_timing = t/float(counter);
        }
    }

};
}

struct painting_structure
{
    bool activated = false;

    int display_weights = 0; //0-nothing, 1-flappy_weights, 2-squashy_weights

    float radius = 0.1f;
    float threshold_percentage = 0.3f;
    float value = 1.0f;

};

enum gui_parameters_display_type {display_sphere, display_character, display_cylinder_bending, display_cylinder_translate, display_cylinder_linear, display_cylinder_sinusoidal_translate, display_cylinder_sinusoidal_bend, display_cylinder_sinusoidal_twist,  display_rondinella, display_bar, display_girafe, display_spot, display_flower, display_dragon, display_snail, display_custom};
struct gui_parameters
{
    int display_type = display_sphere;

    bool display_skeleton_bones = true;
    bool display_skeleton_joints = true;
    bool display_skeleton_frames = true;
    bool display_skeleton_pyramid = true;
    bool display_center_of_mass = false;
    float frame_scaling = 1.0f;
    bool display_mesh = true;
    bool display_rest_pose = false;
    bool display_wireframe = false;
    bool display_texture = true;
    bool dual_quaternion = false;
    bool interpolate = true;
    bool symmetry = true;
    bool fake_speed = false;
    bool curved_trajectory = true;

    bool display_deformation_arrows = false;
    bool display_deformation_target = false;
    int display_deformation_arrow_vertex_offset = 1;

    bool display_deformed_surface = true;
    bool display_joint_linear_speed = false;
    bool display_joint_angular_speed = false;
    bool display_joint_linear_acceleration = false;
    bool display_joint_angular_acceleration = false;
    bool display_joint_centripetal_acceleration = false;
    bool display_joint_tangential_acceleration = false;
    bool display_joint_net_acceleration = false;
    bool display_joint_deformation_vector = false;
    bool display_vertex_to_bone_correspondance = false;
    bool unfiltered_acceleration = false;
    bool smoothstep_followthrough = false;

    bool animation = false;
    bool looped_animation = true;
    bool x_ray = true;
    int type_deformation = 1; //0-translation, 1-rotation
    int squash_around = 0; //0-axis, 1-center
    int squash_detection_mode = 0; //0-perpendicular detection, 1-parallel detecction, 2-reverse engineered detection
    int deformation_mode = 0; //0-velocity, 1-acceleration
    int deformation_mode_squash = 0; //0-velocity, 1-acceleration // 
    int deformation_mode_flappy_translation = 0; //0-velocity, 1-acceleration // 
    int deformation_mode_flappy_rotation = 0; //0-velocity, 1-acceleration // 
    //int deformation_mode_drag = 0; //0-unified, 1-decomposed //
    int deformation_mode_bendy_stretch = 1; // 0-omnidirectional scale, 2-controlled scale
    int deformation_sequence = 0; // 0-parallel, 1-sequential (bendy_stretch->flappy),
    int deformer_affects = 1; //0-local, 1-global (are the deformer parameters local to each joint or global to the entire skeleton?)
    int squash_deformation_parameter = 0; //0-velocity, 1-acceleration;

    float flappy_max_angle_deg = 180.0f / 6.0f;
    float flappy_max_translation = 2.5f;
    float squashing_max = 1.0f;
    float stretch_max = 1.0f;
    float lift_max = 1.0f;
    float follow_through_threshold = 0.5f;
    float stretch_quad_coef = 1.0f;
    float lift_quad_coef = 1.0f;
    


    bool record_anim = false;

    painting_structure painting;

};

enum button_click_type { none, left, right };


struct picking_structure {
    bool is_selected;

    int joint_hover = -1;

    int selected_joint;
    vcl::vec3 p_clicked;
    vcl::vec3 p_current;
    button_click_type click_button = none;
    int selected_joint_true;

    bool picked_center_of_mass = false;


    int painting_selected_vertex = -1;
};

struct scene_model : scene_base
{
    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void set_gui();
    void mouse_move(scene_structure& scene, GLFWwindow* window);
    void mouse_click(scene_structure& scene, GLFWwindow* window, int button, int action, int mods);
    void mouse_scroll(scene_structure& scene, GLFWwindow* window, float x_offset, float y_offset);

    void keyboard_input(scene_structure& scene, GLFWwindow* window, int key, int scancode, int action, int mods);
    void generate_groundplane(const vcl::vec3& origin, float width, float depth);
    skeleton_structure skeleton; // Encode skeleton, hierarchy, anim, etc.
    skinning_structure skinning; // Encode skinning weights, rest pose mesh, deformed mesh
    vcl::mesh_drawable character_visual; // Displayable final mesh


    vcl::curve_drawable painting_cursor;

    std::map<int,int> symmetrical_joint;



    picking_structure picking;


    // temporary skeleton used during interactive deformation (interactive update of the local transformation for each joint)
    vcl::buffer<joint_geometry> skeleton_local_interactive_deformation;
    // The skeleton which is currently displayed and used for the skinning
    vcl::buffer<joint_geometry> skeleton_local_current;
    vcl::buffer<joint_geometry> skeleton_local_current_before_deformation;

    vcl::buffer<joint_speed> skeleton_speed_per_joint;
    vcl::buffer<joint_speed> skeleton_fake_speed_per_joint;

    vcl::buffer<vcl::vec3> triangle_center;
    vcl::buffer<float> triangle_area;
    void update_center_of_mass();

    vcl::vec3 position_center_of_mass(int joint);



    vcl::buffer<vcl::buffer<int> > skeleton_son_connectivity;
    vcl::buffer<bone_correspondance> vertex_to_bone_correspondance;


    vcl::buffer<joint_geometry> skeleton_current;
    vcl::buffer<joint_geometry> skeleton_rest_pose;

    vcl::buffer<vcl::speed_tracker> skeleton_joint_speed;
    //vcl::buffer<vcl::speed_tracker> vertex_speed;
    vcl::buffer<vcl::rotation_tracker> skeleton_joint_rotation_speed;


    std::map<std::string, vcl::timer_tic_toc> timer_measurement;


    vcl::buffer<vcl::buffer< int > > rig_extended_to_ancestor_joint; // per-vertex, per number of joint
    vcl::buffer<vcl::buffer< float > > rig_extended_to_ancestor_weight;
    vcl::buffer<vcl::buffer< int > > vertex_depending_on_joint; // (opposite of rig_extended_to_ancestor_joint) per joint - per number of vertex
    vcl::buffer<vcl::buffer< float > > vertex_weight_depending_on_joint; // (opposite of rig_extended_to_ancestor_weight) per joint - per number of vertex
    vcl::buffer<vcl::buffer<int> > triangle_depending_on_joint; // per joint per triangle
    vcl::buffer<vcl::buffer<int> > triangle_around_vertex;
    vcl::buffer< vcl::vec3 > center_of_mass_per_joint;
    vcl::buffer< vcl::vec3 > center_of_mass_per_joint_manual_offset;

    vcl::buffer<vcl::vec3> save_skinning;

    vcl::buffer<vcl::buffer<float> > skinning_weights_per_joint_per_vertex; // full weights stored as weights[joint][vertex]

    // recording
    vcl::buffer<vcl::vec3> record_position;
    vcl::buffer<vcl::quaternion> record_rotation;
    int recorded_joint = 0;
    float record_dt;
    vcl::timer_event timer_recording;
    float local_time_record;
    std::set<int> record_joint_fixed;

    vcl::buffer<vcl::vec3> deformation_per_vertex;



    // Squashy
    //vcl::buffer<vcl::vec3> skeleton_speed; // to remove
    //vcl::buffer<vcl::vec3> skeleton_acceleration; // to remove



    //vcl::buffer<vcl::velocity_tracker_structure<vcl::vec3> > vertex_velocity; //to-remove

    //vcl::buffer<vcl::velocity_tracker_structure<vcl::vec3> > skeleton_velocity_tracker; // to-remove
    //vcl::buffer<vcl::quaternion_tracker_structure > skeleton_angular_velocity_tracker; // to-remove

    //vcl::buffer<vcl::vec3> previous_angular_velocity; // to-remove
    //vcl::buffer<vcl::vec3> previous_speed; // to-remove

    bool basic_flapping = true; // to-remove
    bool cylinder_flapping = false; // to-remove
    bool display_angular_speed = false; // to-remove

    bool is_interactive = true;
    bool is_flapping = true;
    bool is_speed = false;
    bool is_acceleration = true;
    bool is_rotating = false;
    bool is_backface_deformation_prevented = true;

    vcl::buffer<float> weight_flappy;
    vcl::buffer<float> weight_squashy;

    vcl::quaternion qq; // to-remove

    float centrepetal_acceleration_scale = 1.0f;
    float tangential_acceleration_scale = 1.0f;

    float flapping_power_bend_global = 1.0f;
    vcl::buffer<float> flapping_power_bend_buffer;
    float flapping_power_twist_global = 1.0f;
    vcl::buffer<float> flapping_power_twist_buffer;
    float flapping_power_translate_global = 1.0f;
    vcl::buffer<float> flapping_power_translate_buffer;
    float follow_through_power_translate_global = 0.1f;
    vcl::buffer<float> follow_through_power_translate_buffer;
    float flapping_power_unified_global = 1.0f;
    vcl::buffer<float> flapping_power_unified_buffer;
    float stretch_power_global = 0.0f;
    vcl::buffer<float> bendy_stretch_power_buffer;
    float lift_power_global = 0.0f;
    vcl::buffer<float> lift_power_buffer;
    float acceleration_power_tangential = 1.0f;
    float follow_through_power_rotation_global = 0.1f;
    vcl::buffer<float> follow_through_power_rotation_buffer;
    //float flapping_power = 1.0f;
    //float flapping_bias = 0.5f; // 0: bend, 1: twist
    float squashing_power_global = 1.0f;
    vcl::buffer<float> squashing_power_buffer;// = 1.0f;  // 1 value per joint; 



    void velocity_skinning(float magnitude);
    void flappy_skinning_old();
    void resize_structure(); // resize all skeleton when changing states
    void adapt_skeleton_interactive();
    void apply_deformation_on_skeleton();
    void paint_vertices_around_picked_point();
    void update_painted_color();
    void diffuse_weight();
    void generate_fake_speed();

    vcl::buffer<std::set<unsigned int> > one_ring;

    scene_structure scene;
    std::map<std::string,GLuint> shaders;

    vcl::segment_drawable_immediate_mode segment_drawer;
    vcl::mesh_drawable sphere_visual;
    vcl::mesh_drawable groundplane_visual;
    vcl::mesh_drawable pyramid_skeleton_visual;
    GLuint shader_mesh;
    GLuint shader_groundplane;

    vcl::mesh_drawable frame;

    gui_parameters gui_param;

    vcl::timer_interval timer_skeleton;
    vcl::timer_basic timer;

    arrow_drawable arrow;
    line_drawable arrow_line;

    vcl::vec2 cursor_prev;

    int picked_object = 0;
    int real_texture_id = 0;



    unsigned int depthMapFBO;
    const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
    unsigned int depthMap;
    vcl::mesh_drawable quad;
    unsigned int rbo;


};


#endif
