// #include "../src/hexmap/enums.hpp"
// #include "../src/hexmap/icosahedron.hpp"
// #include "../src/hexmap/point3.hpp"
// #include <iostream>

// #define GLFW_INCLUDE_VULKAN // or point to vulkan.h here
// #include <GLFW/glfw3.h>

// // not using GLM
// // #define GLM_FORCE_RADIANS
// // #define GLM_FORCE_DEPTH_ZERO_TO_ONE
// // #include <glm/vec4.hpp>
// // #include <glm/mat4x4.hpp>

// void glfw_error_callback(int error, const char *description) {
//   // fprintf(stderr, "Error: %s\n", description);
//   std::cout << "\nglfw error received, error: " << std::to_string(error)
//             << ", description: " << description;
// }

// int main() {

// TODO: setup with vulkan, might need to make separate standalone app that
// relies on hexmap to generate points
//.

//   if (!glfwVulkanSupported()) {
//     std::cout << "\nvulkan not supported, ending program";
//     return 0;
//   }

//   // PFN_vkCreateInstance pfnCreateInstance =
//   //     (PFN_vkCreateInstance)glfwGetInstanceProcAddress(NULL,
//   // "vkCreateInstance");

//   // PFN_vkCreateDevice pfnCreateDevice =
//   //     (PFN_vkCreateDevice)glfwGetInstanceProcAddress(instance,
//   //                                                    "vkCreateDevice");

//   // PFN_vkGetDeviceProcAddr pfnGetDeviceProcAddr =
//   //     (PFN_vkGetDeviceProcAddr)glfwGetInstanceProcAddress(
//   //         instance, "vkGetDeviceProcAddr");

//   glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
//   GLFWwindow *window = glfwCreateWindow(640, 480, "Window Title", NULL,
//   NULL);

//   VkSurfaceKHR surface;
//   VkResult err = glfwCreateWindowSurface(instance, window, NULL, &surface);
//   if (err) {
//     // Window surface creation failed
//     std::cout << "\nfailed to create vk window surface";
//   }

//   // if (!glfwInit()) {
//   //   std::cout << "\nfailed to initialize glfw, ending program";
//   //   return 0;
//   // }

//   // glfwSetErrorCallback(glfw_error_callback);

//   // GLFWwindow *window = glfwCreateWindow(500, 500, "window test", NULL,
//   NULL);
//   // if (!window) {
//   //   std::cout << "\nfailed to create glfw window, ending program";
//   //   glfwTerminate();
//   //   return 0;
//   // }

//   vkDestroySurfaceKHR()

//   // glfwTerminate();
// }
