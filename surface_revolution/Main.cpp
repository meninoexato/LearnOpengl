#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "shaders.h"
#include <iostream>

#include <vector>
#include <cmath>
#include <glm/glm.hpp>

using point_t = glm::vec3;



std::vector<point_t> Circle(point_t centro, float raio, int lados)
{
    std::vector<point_t> vertices;
    vertices.reserve(lados * 3);

    for (int i = 0; i < lados; i++)
    {
        float angle = (2.0f * 3.14159265f * i) / lados;
        float x = centro.x + raio * std::cos(angle);
        float y = centro.y + raio * std::sin(angle);
        float z = centro.z;
        
        vertices.push_back(centro);
        vertices.push_back({ x, y, z });

        float next_angle = (2.0f * 3.14159265f * (i + 1)) / lados;
		x = centro.x + raio * std::cos(next_angle) ;
		y =  centro.y + raio * std::sin(next_angle);
        z = centro.z;

		vertices.push_back({ x, y, z });           
    }
	return vertices;
}


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

int main()
{
    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // build and compile our shader zprogram
    Shader ourShader("Vcolor.glsl", "Fcolor.glsl");

    std::vector<point_t> circleVertices = Circle({ 0.0f, 0.0f, 0.0f }, 0.5f, 30);

 
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
   
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 
        circleVertices.size() * sizeof(point_t), 
        circleVertices.data(),
        GL_STATIC_DRAW);

  
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);


    ourShader.use();


    // render loop
    while (!glfwWindowShouldClose(window))
    {
        // input
        processInput(window);

        // render
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);


        // render container
        ourShader.use();
        glBindVertexArray(VAO);
        
        ourShader.setVec3("ourColor", 1.0f, 0.5f, 0.2f);
        //glPointSize(2.0f);
        //glDrawArrays(GL_POINTS, 0, circleVertices.size());
        //glDrawArrays(GL_LINE_LOOP, 0, circleVertices.size());
        glDrawArrays(GL_TRIANGLE_FAN, 0, circleVertices.size());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glfwTerminate();

    return 0;
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}