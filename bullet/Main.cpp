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


#include <btBulletDynamicsCommon.h>

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
	// Initialize GLFW
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
		SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL - Bullet Physics 2.89", nullptr, nullptr);

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

	// Load OpenGL functions
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD\n";
		glfwDestroyWindow(window);
		glfwTerminate();
		return -1;
	}

	glEnable(GL_DEPTH_TEST);

	// ---------------------------------------------------------
	// Bullet Physics initialization
	// ---------------------------------------------------------

	// Collision configuration: defines collision algorithms and their settings.
	btDefaultCollisionConfiguration* collisionConfiguration =
		new btDefaultCollisionConfiguration();

	// Dispatcher: manages collision detection between pairs of objects.
	btCollisionDispatcher* dispatcher =
		new btCollisionDispatcher(collisionConfiguration);

	// Broadphase: quickly identifies pairs of objects that may collide.
	btBroadphaseInterface* overlappingPairCache =
		new btDbvtBroadphase();

	// Constraint solver: resolves contacts and constraints.
	btSequentialImpulseConstraintSolver* solver =
		new btSequentialImpulseConstraintSolver();

	// Dynamics world: central object that manages the physics simulation.
	btDiscreteDynamicsWorld* dynamicsWorld =
		new btDiscreteDynamicsWorld(
			dispatcher,
			overlappingPairCache,
			solver,
			collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0.0f, -9.81f, 0.0f));

	// ---------------------------------------------------------
	// Create physics objects
	// ---------------------------------------------------------

	// FLOOR: static body (mass = 0).
	btCollisionShape* floorShape =
		new btBoxShape(btVector3(10.0f, 0.25f, 10.0f));

	btTransform floorTransform;
	floorTransform.setIdentity();
	floorTransform.setOrigin(btVector3(0.0f, -0.25f, 0.0f));

	btDefaultMotionState* floorMotionState =
		new btDefaultMotionState(floorTransform);

	btRigidBody::btRigidBodyConstructionInfo floorBodyCI(
		0.0f,
		floorMotionState,
		floorShape,
		btVector3(0.0f, 0.0f, 0.0f));

	btRigidBody* floorBody = new btRigidBody(floorBodyCI);
	floorBody->setFriction(0.5f);
	dynamicsWorld->addRigidBody(floorBody);

	// CUBE: dynamic body (mass = 1).
	btCollisionShape* cubeShape =
		new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));

	btTransform cubeTransform;
	cubeTransform.setIdentity();
	cubeTransform.setOrigin(btVector3(0.0f, 5.0f, 0.0f));

	btVector3 cubeInertia(0.0f, 0.0f, 0.0f);
	const btScalar cubeMass = 1.0f;
	cubeShape->calculateLocalInertia(cubeMass, cubeInertia);

	btDefaultMotionState* cubeMotionState =
		new btDefaultMotionState(cubeTransform);

	btRigidBody::btRigidBodyConstructionInfo cubeBodyCI(
		cubeMass,
		cubeMotionState,
		cubeShape,
		cubeInertia);

	btRigidBody* cubeBody = new btRigidBody(cubeBodyCI);
	cubeBody->setFriction(0.5f);
	cubeBody->setRestitution(0.1f);
	dynamicsWorld->addRigidBody(cubeBody);

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
	// Render loop
	// ---------------------------------------------------------
	while (!glfwWindowShouldClose(window))
	{
		const float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Avoid an excessively large simulation step after a pause/stall.
		const btScalar physicsDeltaTime =
			static_cast<btScalar>(std::min(deltaTime, 0.1f));

		// Bullet uses a fixed internal timestep (1/60 s) and can perform
		// up to 10 substeps to catch up with the elapsed frame time.
		dynamicsWorld->stepSimulation(physicsDeltaTime, 10, 1.0f / 60.0f);

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

		// Draw the floor. The OpenGL cube has unit dimensions, so scale it
		// to 20 x 0.5 x 20 to match the Bullet box half-extents.
		glm::mat4 floorModel(1.0f);
		floorModel = glm::translate(floorModel, glm::vec3(0.0f, -0.25f, 0.0f));
		floorModel = glm::scale(floorModel, glm::vec3(20.0f, 0.5f, 20.0f));
		ourShader.setMat4("model", floorModel);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		// Read the dynamic body's transform from Bullet.
		btTransform bulletCubeTransform;
		cubeBody->getMotionState()->getWorldTransform(bulletCubeTransform);

		const btVector3 cubePosition = bulletCubeTransform.getOrigin();
		const btQuaternion cubeRotation = bulletCubeTransform.getRotation();

		glm::vec3 position(
			static_cast<float>(cubePosition.getX()),
			static_cast<float>(cubePosition.getY()),
			static_cast<float>(cubePosition.getZ()));

		// GLM quaternion constructor order: w, x, y, z.
		glm::quat rotation(
			static_cast<float>(cubeRotation.getW()),
			static_cast<float>(cubeRotation.getX()),
			static_cast<float>(cubeRotation.getY()),
			static_cast<float>(cubeRotation.getZ()));

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
	// Cleanup Bullet objects
	// Remove rigid bodies from the world before deleting them.
	// ---------------------------------------------------------
	dynamicsWorld->removeRigidBody(cubeBody);
	dynamicsWorld->removeRigidBody(floorBody);

	delete cubeBody;
	delete cubeMotionState;
	delete cubeShape;

	delete floorBody;
	delete floorMotionState;
	delete floorShape;

	// Destroy the world and its supporting components.
	delete dynamicsWorld;
	delete solver;
	delete overlappingPairCache;
	delete dispatcher;
	delete collisionConfiguration;

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