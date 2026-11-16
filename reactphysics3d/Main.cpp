#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "stb_image.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include "shaders.h"
#include "camera.h"

#include <reactphysics3d/reactphysics3d.h>

#include <iostream>
#include <algorithm>


// Forward declarations
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);


// Window settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;


// Camera
Camera camera(glm::vec3(0.0f, 3.0f, 10.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

\
// Timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;


int main()
{
   if (!glfwInit())
   {
      std::cout << "Failed to initialize GLFW\n";
      return -1;
   }

   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
   glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

   GLFWwindow* window = glfwCreateWindow(
      SCR_WIDTH, SCR_HEIGHT,
      "LearnOpenGL - ReactPhysics3D 0.10.2", nullptr, nullptr);

   if (window == nullptr)
   {
      std::cout << "Failed to create GLFW window\n";
      glfwTerminate();
      return -1;
   }

   glfwMakeContextCurrent(window);
   glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
   glfwSetCursorPosCallback(window, mouse_callback);
   glfwSetScrollCallback(window, scroll_callback);
   glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

   if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
   {
      std::cout << "Failed to initialize GLAD\n";
      glfwDestroyWindow(window);
      glfwTerminate();
      return -1;
   }

   glEnable(GL_DEPTH_TEST);

   // ---------------------------------------------------------
   // ReactPhysics3D initialization
   // ---------------------------------------------------------

   // PhysicsCommon is the factory and memory manager for RP3D.
   reactphysics3d::PhysicsCommon physicsCommon;

   reactphysics3d::PhysicsWorld::WorldSettings worldSettings;
   worldSettings.gravity = reactphysics3d::Vector3(0.0, -9.81, 0.0);

   reactphysics3d::PhysicsWorld* physicsWorld =
      physicsCommon.createPhysicsWorld(worldSettings);

   // ---------------------------------------------------------
   // Create physics objects
   // ---------------------------------------------------------

   // FLOOR: static body, box half-extents = (10, 0.25, 10).
   reactphysics3d::BoxShape* floorShape =
      physicsCommon.createBoxShape(reactphysics3d::Vector3(10.0, 0.25, 10.0));

   reactphysics3d::Transform floorTransform(
      reactphysics3d::Vector3(0.0, -0.25, 0.0),
      reactphysics3d::Quaternion::identity());

   reactphysics3d::RigidBody* floorBody =
      physicsWorld->createRigidBody(floorTransform);

   floorBody->setType(reactphysics3d::BodyType::STATIC);

   reactphysics3d::Collider* floorCollider =
      floorBody->addCollider(floorShape, reactphysics3d::Transform::identity());

   floorCollider->getMaterial().setFrictionCoefficient(0.5);

   // CUBE: dynamic body, box half-extents = (0.5, 0.5, 0.5).
   reactphysics3d::BoxShape* cubeShape =
      physicsCommon.createBoxShape(reactphysics3d::Vector3(0.5, 0.5, 0.5));

   reactphysics3d::Transform cubeTransform(
      reactphysics3d::Vector3(0.0, 5.0, 0.0),
      reactphysics3d::Quaternion::identity());

   reactphysics3d::RigidBody* cubeBody =
      physicsWorld->createRigidBody(cubeTransform);

   cubeBody->setType(reactphysics3d::BodyType::DYNAMIC);
   cubeBody->setMass(1.0);

   reactphysics3d::Collider* cubeCollider =
      cubeBody->addCollider(cubeShape, reactphysics3d::Transform::identity());

   cubeCollider->getMaterial().setFrictionCoefficient(0.5);
   cubeCollider->getMaterial().setBounciness(0.1);

   // ---------------------------------------------------------
   // OpenGL shader
   // ---------------------------------------------------------
   Shader ourShader("Vcolor.glsl", "Fcolor.glsl");

   // Unit cube: position (x, y, z) and texture coordinates (u, v).
   float vertices[] = {
       -0.5f,-0.5f,-0.5f, 0.0f,0.0f,  0.5f,-0.5f,-0.5f, 1.0f,0.0f,
        0.5f, 0.5f,-0.5f, 1.0f,1.0f,  0.5f, 0.5f,-0.5f, 1.0f,1.0f,
       -0.5f, 0.5f,-0.5f, 0.0f,1.0f, -0.5f,-0.5f,-0.5f, 0.0f,0.0f,

       -0.5f,-0.5f, 0.5f, 0.0f,0.0f,  0.5f,-0.5f, 0.5f, 1.0f,0.0f,
        0.5f, 0.5f, 0.5f, 1.0f,1.0f,  0.5f, 0.5f, 0.5f, 1.0f,1.0f,
       -0.5f, 0.5f, 0.5f, 0.0f,1.0f, -0.5f,-0.5f, 0.5f, 0.0f,0.0f,

       -0.5f, 0.5f, 0.5f, 1.0f,0.0f, -0.5f, 0.5f,-0.5f, 1.0f,1.0f,
       -0.5f,-0.5f,-0.5f, 0.0f,1.0f, -0.5f,-0.5f,-0.5f, 0.0f,1.0f,
       -0.5f,-0.5f, 0.5f, 0.0f,0.0f, -0.5f, 0.5f, 0.5f, 1.0f,0.0f,

        0.5f, 0.5f, 0.5f, 1.0f,0.0f,  0.5f, 0.5f,-0.5f, 1.0f,1.0f,
        0.5f,-0.5f,-0.5f, 0.0f,1.0f,  0.5f,-0.5f,-0.5f, 0.0f,1.0f,
        0.5f,-0.5f, 0.5f, 0.0f,0.0f,  0.5f, 0.5f, 0.5f, 1.0f,0.0f,

       -0.5f,-0.5f,-0.5f, 0.0f,1.0f,  0.5f,-0.5f,-0.5f, 1.0f,1.0f,
        0.5f,-0.5f, 0.5f, 1.0f,0.0f,  0.5f,-0.5f, 0.5f, 1.0f,0.0f,
       -0.5f,-0.5f, 0.5f, 0.0f,0.0f, -0.5f,-0.5f,-0.5f, 0.0f,1.0f,

       -0.5f, 0.5f,-0.5f, 0.0f,1.0f,  0.5f, 0.5f,-0.5f, 1.0f,1.0f,
        0.5f, 0.5f, 0.5f, 1.0f,0.0f,  0.5f, 0.5f, 0.5f, 1.0f,0.0f,
       -0.5f, 0.5f, 0.5f, 0.0f,0.0f, -0.5f, 0.5f,-0.5f, 0.0f,1.0f
   };

   unsigned int VBO, VAO;
   glGenVertexArrays(1, &VAO);
   glGenBuffers(1, &VBO);

   glBindVertexArray(VAO);
   glBindBuffer(GL_ARRAY_BUFFER, VBO);
   glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
      5 * sizeof(float), (void*)0);
   glEnableVertexAttribArray(0);

   glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
      5 * sizeof(float), (void*)(3 * sizeof(float)));
   glEnableVertexAttribArray(1);

   // ---------------------------------------------------------
   // Load textures
   // ---------------------------------------------------------
   unsigned int texture1, texture2;
   int width, height, nrChannels;

   stbi_set_flip_vertically_on_load(true);

   glGenTextures(1, &texture1);
   glBindTexture(GL_TEXTURE_2D, texture1);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   unsigned char* data =
      stbi_load("../textura/container.jpg", &width, &height, &nrChannels, 0);

   if (data)
   {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
         width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
      glGenerateMipmap(GL_TEXTURE_2D);
   }
   else
   {
      std::cout << "Failed to load container.jpg\n";
   }
   stbi_image_free(data);

   glGenTextures(1, &texture2);
   glBindTexture(GL_TEXTURE_2D, texture2);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   data = stbi_load("../textura/awesomeface.png",
      &width, &height, &nrChannels, STBI_rgb_alpha);

   if (data)
   {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
         width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
      glGenerateMipmap(GL_TEXTURE_2D);
   }
   else
   {
      std::cout << "Failed to load awesomeface.png\n";
   }
   stbi_image_free(data);

   ourShader.use();
   ourShader.setInt("texture1", 0);
   ourShader.setInt("texture2", 1);

   // ---------------------------------------------------------
   // Fixed-step timing for ReactPhysics3D
   // ---------------------------------------------------------
   const float physicsTimeStep = 1.0f / 60.0f;
   float physicsAccumulator = 0.0f;

   // ---------------------------------------------------------
   // Render loop
   // ---------------------------------------------------------
   while (!glfwWindowShouldClose(window))
   {
      const float currentFrame = static_cast<float>(glfwGetTime());
      deltaTime = currentFrame - lastFrame;
      lastFrame = currentFrame;

      // Prevent a large catch-up after a pause or debugger breakpoint.
      const float frameTime = std::min(deltaTime, 0.1f);
      physicsAccumulator += frameTime;

      // RP3D takes a fixed time step per update.
      while (physicsAccumulator >= physicsTimeStep)
      {
         physicsWorld->update(physicsTimeStep);
         physicsAccumulator -= physicsTimeStep;
      }

      processInput(window);

      glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture1);
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, texture2);

      ourShader.use();

      glm::mat4 projection = glm::perspective(
         glm::radians(camera.Zoom),
         static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT),
         0.1f, 100.0f);
      ourShader.setMat4("projection", projection);

      glm::mat4 view = camera.GetViewMatrix();
      ourShader.setMat4("view", view);

      glBindVertexArray(VAO);

      // Draw the floor. Unit cube scaled to 20 x 0.5 x 20.
      glm::mat4 floorModel(1.0f);
      floorModel = glm::translate(floorModel, glm::vec3(0.0f, -0.25f, 0.0f));
      floorModel = glm::scale(floorModel, glm::vec3(20.0f, 0.5f, 20.0f));
      ourShader.setMat4("model", floorModel);
      glDrawArrays(GL_TRIANGLES, 0, 36);

      // Read the dynamic body's transform from ReactPhysics3D.
      const reactphysics3d::Transform& rp3dTransform = cubeBody->getTransform();
      const reactphysics3d::Vector3& rp3dPosition = rp3dTransform.getPosition();
      const reactphysics3d::Quaternion& rp3dRotation = rp3dTransform.getOrientation();

      glm::vec3 position(
         static_cast<float>(rp3dPosition.x),
         static_cast<float>(rp3dPosition.y),
         static_cast<float>(rp3dPosition.z));

      // GLM quaternion constructor order: w, x, y, z.
      glm::quat rotation(
         static_cast<float>(rp3dRotation.w),
         static_cast<float>(rp3dRotation.x),
         static_cast<float>(rp3dRotation.y),
         static_cast<float>(rp3dRotation.z));

      glm::mat4 cubeModel(1.0f);
      cubeModel = glm::translate(cubeModel, position);
      cubeModel *= glm::mat4_cast(rotation);

      ourShader.setMat4("model", cubeModel);
      glDrawArrays(GL_TRIANGLES, 0, 36);

      glfwSwapBuffers(window);
      glfwPollEvents();
   }

   // ---------------------------------------------------------
   // Cleanup OpenGL resources
   // ---------------------------------------------------------
   glDeleteVertexArrays(1, &VAO);
   glDeleteBuffers(1, &VBO);
   glDeleteTextures(1, &texture1);
   glDeleteTextures(1, &texture2);

   // ---------------------------------------------------------
   // Cleanup ReactPhysics3D
   // ---------------------------------------------------------
   // Destroy the world first. It also destroys bodies and their colliders.
   physicsCommon.destroyPhysicsWorld(physicsWorld);

   // Shapes were created by PhysicsCommon and must be destroyed by it.
   physicsCommon.destroyBoxShape(cubeShape);
   physicsCommon.destroyBoxShape(floorShape);

   glfwDestroyWindow(window);
   glfwTerminate();

   return 0;
}





// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
}


// glfw: whenever the window size changed (by OS or user resize) this callback function execute
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
	float xpos = static_cast<float>(xposIn);
	float ypos = static_cast<float>(yposIn);

	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(static_cast<float>(yoffset));
}