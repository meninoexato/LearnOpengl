#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "shaders.h"
#include <iostream>


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <cmath>





using point_t = glm::vec3;


std::vector<point_t> Esfera(point_t centro, float raio, int stacks, int slices);
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

    glEnable(GL_DEPTH_TEST);

    // build and compile our shader zprogram
    Shader ourShader("Vcolor.glsl", "Fcolor.glsl");

    std::vector<point_t> esfera = Esfera({ 0.3f, 0.3f, 0.3f }, 0.5f, 30, 30);
 
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
   
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 
        esfera.size() * sizeof(point_t), 
        esfera.data(),
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
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        ourShader.use();

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);
        glm::mat4 projection = glm::mat4(1.0f);

     
        model = glm::rotate(model, (float)glfwGetTime() * glm::radians(50.0f), glm::vec3(0.5f, 1.0f, 0.0f));
        view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
        projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

        // 2. Envio limpo para o shader
        ourShader.setMat4("projection", projection);
        ourShader.setMat4("view", view);
        ourShader.setMat4("model", model);
        ourShader.setVec3("ourColor", 1.0f, 0.5f, 0.2f);


        glBindVertexArray(VAO);
        //glPointSize(3.0f);
        //glDrawArrays(GL_POINTS, 0, esfera.size());
        glDrawArrays(GL_LINE_LOOP, 0, esfera.size());
        //glDrawArrays(GL_TRIANGLES, 0, esfera.size());

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


std::vector<point_t> Esfera(point_t centro, float raio, int stacks, int slices)
{
    std::vector<point_t> vertices;
    vertices.reserve(stacks * slices * 6);

    std::vector<std::vector<point_t>> grid(stacks + 1, std::vector<point_t>(slices + 1));

    {
        for (int i = 0; i <= stacks; i++)
        {
            float phi = (3.14159265f * i) / stacks;

            for (int j = 0; j <= slices; j++)
            {
                float theta = (2.0f * 3.14159265f * j) / slices;
                float x = centro.x + raio * std::sin(phi) * std::cos(theta);
                float y = centro.y + raio * std::cos(phi);
                float z = centro.z + raio * std::sin(phi) * std::sin(theta);

                grid[i][j] = { x, y, z };

            }
        }


        for (int i = 0; i < stacks; i++)
        {
            for (int j = 0; j < slices; j++)
            {
                point_t p00 = grid[i][j];
                point_t p10 = grid[i + 1][j];
                point_t p11 = grid[i + 1][j + 1];
                point_t p01 = grid[i][j + 1];

                vertices.push_back(p00);
                vertices.push_back(p10);
                vertices.push_back(p11);

                vertices.push_back(p00);
                vertices.push_back(p11);
                vertices.push_back(p01);
            }
        }


        return vertices;


    }
}
