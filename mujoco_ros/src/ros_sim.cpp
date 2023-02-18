#include <ros/ros.h>
#include <cstdio>
#include <cstring>
#include <glfw_dispatch.h>

#include <GLFW/glfw3.h>
#include <mujoco.h>
#include "ctrl_interface.h"
#include "physics.h"


extern void positionCB(const daedalus_msgs::TeensyMsg::ConstPtr& msg);
extern void initialize_forces();
extern void set_forces();
extern void send_object_pose();


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

typedef struct BodyPos_ {
  double x;
  double y;
  double z;
} BodyPos;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// main function
int main(int argc, char** argv) {
  // check command-line arguments
  if (argc!=5 && argc!=3) {
    std::printf(" USAGE:  basic modelfile fixed_camera(true/false)");
    return 0;
  }
  ros::init(argc, argv, "mujoco_interface");
  
  init_ros_node();

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    mju_error_s("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  // Reduce lag by reducing window size (1920, 1080) or (720, 480)
  GLFWwindow* window = glfwCreateWindow(720, 480, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // Set camera position
  if (strcmp(argv[2], "true") == 0)
  {
    // TODO: move these to ros parameters
    cam.lookat[0] = 0;
    cam.lookat[1] = 0;
    cam.lookat[2] = 0.1;

    cam.azimuth = -90;
    cam.elevation = -10;
    cam.distance = 0.4;
  } else {
  // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
  }
  
  

  mjcb_control = &ctrl_robot;

  // get framebuffer viewport
  mjrRect viewport = mjr_maxViewport(&con);
  int W = viewport.width;
  int H = viewport.height;
  init_frame_pub(W, H);
  init_robot_state();
  
  

  // Depth cam data
  unsigned char* rgb = (unsigned char*)std::malloc(3*W*H);
  float* depth = (float*)std::malloc(sizeof(float)*W*H);
  if(!rgb || !depth)
  {
    mju_error("Could not allocate buffers");
  }

  BodyPos* body_positions = (BodyPos*)std::malloc(sizeof(BodyPos)*m->nbody);

  initialize_forces();

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window) && ros::ok()) {

    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0/60.0) {
      mj_step(m, d);
    }


    set_forces();
    ros_reset(); // Checks if reset should occur



    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);


    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // Read Depth Camera
    mjr_readPixels(rgb, depth, viewport, &con);
    send_rgb(rgb);
    send_depth(depth);
    //send_cam_pos(d->cam_xpos, m->cam_quat); Gives depth cam position
    get_cam_pos(&cam.lookat[0], cam.distance, cam.azimuth, cam.elevation);
    send_cam_pos(m->cam_pos, m->cam_quat);
    send_object_state();

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
