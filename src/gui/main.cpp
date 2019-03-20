#include "imgui.h"
#include "imgui_impl_glfw.h"
#include <stdio.h>
#include <iostream>

#include <GL/glew.h>    // This example is using gl3w to access OpenGL functions (because it is small). You may use glew/glad/glLoadGen/etc
#include <GLFW/glfw3.h>

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
}


/*

Options:
    Select options for capture
        - make fragemnts or capture images
        - use gps
    
Start Capturing

Report Status
    - Initializing
    - Capturing
    - Finishing

Display Summary Statistics
    - GPS status
    - Number of frames captured
    - Actual FPS
    - Processing Queue Size
    - Quality of matches


*/



int main(int, char**)
{
    // Setup window
    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) return 1;

    GLFWwindow* window = glfwCreateWindow(1280, 720, "StreetSmarts", NULL, NULL);

    glfwMakeContextCurrent(window);
    glewInit();

    // Setup ImGui binding
    ImGui_ImplGlfw_Init(window, true);

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

   // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplGlfw_NewFrame(1.5);

        ImGui::Begin("Controller");

        if (ImGui::Button("Start")){
            //Begin 
        }

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)");

        ImGui::End();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        //glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        glfwMakeContextCurrent(window);
        ImGui::Render();
        glfwSwapBuffers(window);
    }


    // Cleanup
    ImGui_ImplGlfw_Shutdown();
    glfwTerminate();

    return 0;
}
