#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "stb_image.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>


#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/core/ChTypes.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/collision/ChCollisionSystem.h>

#include "shaders.h"
#include "camera.h"

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
		SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL - Project Chrono 11", nullptr, nullptr);

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
	// ========================================
	// PROJECT CHRONO - PHYSICS SETUP
	// ========================================

	using namespace chrono;

	ChSystemNSC physicsSystem;

	physicsSystem.SetGravitationalAcceleration(
		ChVector3d(0.0, -9.81, 0.0)
	);

	physicsSystem.SetCollisionSystemType(
		ChCollisionSystem::Type::BULLET
	);

	auto material = chrono_types::make_shared<ChContactMaterialNSC>();
	material->SetFriction(0.5);
	material->SetRestitution(0.3);

	// ChBodyEasyBox creates the rigid body and its collision shape.
	// Ground dimensions: 20 x 1 x 20.
	auto ground = chrono_types::make_shared<ChBodyEasyBox>(
		20.0, 1.0, 20.0,
		1000.0,
		false,   // do not create Chrono visualization assets
		true,    // create collision shape
		material
	);

	ground->SetPos(ChVector3d(0.0, -0.5, 0.0));
	ground->SetFixed(true);
	physicsSystem.Add(ground);

	// Dynamic cube: 1 x 1 x 1, density 1 kg/m^3 (mass = 1 kg).
	auto cube = chrono_types::make_shared<ChBodyEasyBox>(
		1.0, 1.0, 1.0,
		1.0,
		false,
		true,
		material
	);

	cube->SetPos(ChVector3d(0.0, 5.0, 0.0));
	physicsSystem.Add(cube);

	std::cout << "Project Chrono initialized. Cube is falling.\n";

	// Fixed timestep keeps the simulation stable and independent
	// from the rendering frame rate.
	const double fixedTimeStep = 1.0 / 120.0;
	double physicsAccumulator = 0.0;

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
	lastFrame = static_cast<float>(glfwGetTime());

	while (!glfwWindowShouldClose(window))
	{
		const float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Prevent a huge simulation step after a pause or breakpoint.
		deltaTime = std::min(deltaTime, 0.1f);

		processInput(window);

		physicsAccumulator += static_cast<double>(deltaTime);
		while (physicsAccumulator >= fixedTimeStep)
		{
			physicsSystem.DoStepDynamics(fixedTimeStep);
			physicsAccumulator -= fixedTimeStep;
		}

		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texture1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, texture2);

		ourShader.use();

		const glm::mat4 projection = glm::perspective(
			glm::radians(camera.Zoom),
			static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT),
			0.1f,
			100.0f
		);
		const glm::mat4 view = camera.GetViewMatrix();

		ourShader.setMat4("projection", projection);
		ourShader.setMat4("view", view);

		// Draw the ground. Its transform matches the Chrono collision box.
		glm::mat4 groundModel(1.0f);
		groundModel = glm::translate(groundModel, glm::vec3(0.0f, -0.5f, 0.0f));
		groundModel = glm::scale(groundModel, glm::vec3(20.0f, 1.0f, 20.0f));
		ourShader.setMat4("model", groundModel);
		glBindVertexArray(VAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		// Position and rotation of the dynamic cube, read from Chrono.
		ChVector3d position = cube->GetPos();

		// Rotação do cubo
		ChQuaternion<> rotation = cube->GetRot();

		// Converter posição para GLM
		glm::vec3 cubePosition(
			static_cast<float>(position.x()),
			static_cast<float>(position.y()),
			static_cast<float>(position.z())
		);

		// Converter quaternion do Chrono para GLM
		glm::quat cubeRotation(
			static_cast<float>(rotation.e0()),
			static_cast<float>(rotation.e1()),
			static_cast<float>(rotation.e2()),
			static_cast<float>(rotation.e3())
		);

		// Criar matriz de transformação
		glm::mat4 model = glm::translate(
			glm::mat4(1.0f),
			cubePosition
		);

		model *= glm::toMat4(cubeRotation);
		ourShader.setMat4("model", model);
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

	// Chrono bodies and system use shared_ptr/RAII.
	// They are released automatically when leaving main().

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