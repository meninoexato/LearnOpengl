#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "shaders.h"
#include <iostream>

#include <vector>
#include <cmath>
#include <functional>  
#include <glm/glm.hpp>


using point_t = glm::vec3;

struct Vertex
{
   point_t   position;
   point_t   normal;
   glm::vec2 uv;
};

std::vector<Vertex> TriangulateSurfaceOfRevolution(
   std::function<point_t(float)> g, float t0, float tf, int slices, int stacks);

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

   // curva geratriz: x = raio, y = altura
   auto parabola = [](float t) -> point_t {
      return point_t(t, t * t, 0.0f);
      };

   std::vector<Vertex> vertices = TriangulateSurfaceOfRevolution(parabola, 0.0f, 0.9f, 32, 32);


   unsigned int VBO, VAO;
   glGenVertexArrays(1, &VAO);
   glGenBuffers(1, &VBO);

   glBindVertexArray(VAO);
   glBindBuffer(GL_ARRAY_BUFFER, VBO);
   glBufferData(GL_ARRAY_BUFFER,
      vertices.size() * sizeof(Vertex),
      vertices.data(),
      GL_STATIC_DRAW);

   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
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
      //glDrawArrays(GL_POINTS, 0, vertices.size());
      //glDrawArrays(GL_LINE_LOOP, 0, vertices.size());
      glDrawArrays(GL_TRIANGLES, 0, vertices.size());

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

std::vector<Vertex> TriangulateSurfaceOfRevolution(
   std::function<point_t(float)> g, float t0, float tf, int slices, int stacks)
{
   std::vector<Vertex> vertices;
   vertices.reserve(stacks * slices * 6);

   std::vector<std::vector<point_t>> grid(stacks + 1, std::vector<point_t>(slices + 1));
   std::vector<std::vector<glm::vec2>> gridUV(stacks + 1, std::vector<glm::vec2>(slices + 1));

   for (int i = 0; i <= stacks; i++)
   {
      float t = t0 + (tf - t0) * i / stacks;
      point_t p = g(t);
      float v = (float)i / stacks;

      for (int j = 0; j <= slices; j++)
      {
         float theta = (2.0f * 3.14159265f * j) / slices;
         float c = std::cos(theta);
         float s = std::sin(theta);

         grid[i][j] = { p.x * c + p.z * s, p.y, -p.x * s + p.z * c };
         gridUV[i][j] = { (float)j / slices, v };
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

         glm::vec2 uv00 = gridUV[i][j];
         glm::vec2 uv10 = gridUV[i + 1][j];
         glm::vec2 uv11 = gridUV[i + 1][j + 1];
         glm::vec2 uv01 = gridUV[i][j + 1];

         // NÃO normalizar ainda — precisa medir o comprimento cru primeiro
         point_t tangenteU = p10 - p00;
         point_t tangenteV = p01 - p00;

         float lenU = glm::length(tangenteU);
         float lenV = glm::length(tangenteV);

         point_t normal;
         if (lenU < 1e-6f || lenV < 1e-6f)
         {
            point_t altU = p11 - p01;
            point_t altV = p11 - p10;
            normal = glm::normalize(glm::cross(altU, altV));
         }
         else
         {
            normal = glm::normalize(glm::cross(tangenteU, tangenteV));
         }

         vertices.push_back({ p00, normal, uv00 });
         vertices.push_back({ p10, normal, uv10 });
         vertices.push_back({ p11, normal, uv11 });

         vertices.push_back({ p00, normal, uv00 });
         vertices.push_back({ p11, normal, uv11 });
         vertices.push_back({ p01, normal, uv01 });
      }
   }

   return vertices;
}
