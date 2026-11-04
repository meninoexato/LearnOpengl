#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "stb_image.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shaders.h"
#include "camera.h"


// Sempre inclua Jolt.h antes dos demais cabeçalhos do Jolt.
#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#include <thread>
#include <iostream>
#include <algorithm>
#include <cstdarg>
#include <cstdio>


// Categorias dos objetos físicos
namespace Layers
{
	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING = 1;
}

// Categorias utilizadas pela BroadPhase
namespace BroadPhaseLayers
{
	static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH::BroadPhaseLayer MOVING(1);
}

// Mapeia ObjectLayer para BroadPhaseLayer
// Mapeia ObjectLayer para BroadPhaseLayer
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
	JPH::uint GetNumBroadPhaseLayers() const override
	{
		return 2;
	}

	JPH::BroadPhaseLayer GetBroadPhaseLayer(
		JPH::ObjectLayer layer) const override
	{
		return layer == Layers::NON_MOVING
			? BroadPhaseLayers::NON_MOVING
			: BroadPhaseLayers::MOVING;
	}

	// Retorna o nome de cada BroadPhaseLayer.
	const char* GetBroadPhaseLayerName(
		JPH::BroadPhaseLayer layer) const override
	{
		if (layer == BroadPhaseLayers::NON_MOVING)
			return "NON_MOVING";

		if (layer == BroadPhaseLayers::MOVING)
			return "MOVING";

		return "UNKNOWN";
	}
};

// Define quais objetos podem colidir com cada BroadPhaseLayer
class ObjectVsBroadPhaseLayerFilterImpl final
	: public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	bool ShouldCollide(
		JPH::ObjectLayer layer,
		JPH::BroadPhaseLayer broadPhaseLayer) const override
	{
		if (layer == Layers::NON_MOVING)
			return broadPhaseLayer == BroadPhaseLayers::MOVING;

		return true;
	}
};

// Define quais categorias de objetos podem colidir entre si
class ObjectLayerPairFilterImpl final
	: public JPH::ObjectLayerPairFilter
{
public:
	bool ShouldCollide(
		JPH::ObjectLayer layer1,
		JPH::ObjectLayer layer2) const override
	{
		return layer1 == Layers::MOVING ||
			layer2 == Layers::MOVING;
	}
};



// Recebe mensagens de log do Jolt Physics
static void TraceImpl(const char* inFMT, ...)
{
	va_list args;
	va_start(args, inFMT);

	vprintf(inFMT, args);
	printf("\n");

	va_end(args);
}





void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 3.0f, 10.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;


// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

int main()
{
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// glfw window creation
	// --------------------
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// tell GLFW to capture our mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


	// glad: load all OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// configure global opengl state
	glEnable(GL_DEPTH_TEST);



	// Inicialização do Jolt Physics
	// -----------------------------------------
	{
		JPH::RegisterDefaultAllocator();

		// Configura a função de log do Jolt
		JPH::Trace = TraceImpl;

		// Inicializa a fábrica de tipos do Jolt
		JPH::Factory::sInstance = new JPH::Factory();

		// Registra os tipos de física
		JPH::RegisterTypes();

		// Configurações do mundo físico
		const JPH::uint maxBodies = 1024;
		const JPH::uint numBodyMutexes = 0;
		const JPH::uint maxBodyPairs = 1024;
		const JPH::uint maxContactConstraints = 1024;

		// Filtros de colisão
		BPLayerInterfaceImpl broadPhaseLayerInterface;
		ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhaseLayerFilter;
		ObjectLayerPairFilterImpl objectLayerPairFilter;

		// Sistema de física
		JPH::PhysicsSystem physicsSystem;

		physicsSystem.Init(
			maxBodies,
			numBodyMutexes,
			maxBodyPairs,
			maxContactConstraints,
			broadPhaseLayerInterface,
			objectVsBroadPhaseLayerFilter,
			objectLayerPairFilter
		);

		// Memória temporária utilizada pelo Jolt
		JPH::TempAllocatorImpl tempAllocator(10 * 1024 * 1024);

		// Sistema de tarefas
		JPH::JobSystemThreadPool jobSystem(
			JPH::cMaxPhysicsJobs,
			JPH::cMaxPhysicsBarriers,
			2
		);
		// Interface para criar e manipular corpos físicos
		JPH::BodyInterface& bodyInterface =
			physicsSystem.GetBodyInterface();



		// -----------------------------------------
	// Criação dos objetos físicos
	// -----------------------------------------

	// CHÃO: objeto estático
		JPH::BoxShapeSettings floorShapeSettings(
			JPH::Vec3(10.0f, 0.25f, 10.0f)
		);

		JPH::ShapeSettings::ShapeResult floorShapeResult =
			floorShapeSettings.Create();

		JPH::ShapeRefC floorShape = floorShapeResult.Get();

		// Posição do chão
		JPH::BodyCreationSettings floorSettings(
			floorShape,
			JPH::RVec3(0.0, -0.25, 0.0),
			JPH::Quat::sIdentity(),
			JPH::EMotionType::Static,
			Layers::NON_MOVING
		);

		JPH::Body* floorBody = bodyInterface.CreateBody(floorSettings);

		bodyInterface.AddBody(
			floorBody->GetID(),
			JPH::EActivation::DontActivate
		);


		// CUBO: objeto dinâmico
		JPH::BoxShapeSettings cubeShapeSettings(
			JPH::Vec3(0.5f, 0.5f, 0.5f)
		);

		JPH::ShapeSettings::ShapeResult cubeShapeResult =
			cubeShapeSettings.Create();

		JPH::ShapeRefC cubeShape = cubeShapeResult.Get();

		// Posição inicial do cubo: 5 unidades acima do chão
		JPH::BodyCreationSettings cubeSettings(
			cubeShape,
			JPH::RVec3(0.0, 5.0, 0.0),
			JPH::Quat::sIdentity(),
			JPH::EMotionType::Dynamic,
			Layers::MOVING
		);

		cubeSettings.mFriction = 0.5f;
		cubeSettings.mRestitution = 0.1f;

		JPH::Body* cubeBody = bodyInterface.CreateBody(cubeSettings);

		bodyInterface.AddBody(
			cubeBody->GetID(),
			JPH::EActivation::Activate
		);




		// build and compile our shader zprogram
		Shader ourShader("Vcolor.glsl", "Fcolor.glsl");

		// set up vertex data (and buffer(s)) and configure vertex attributes
		// ------------------------------------------------------------------
		float vertices[] = {
		-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
		 0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
		-0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

		-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		-0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

		-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
		-0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
		-0.5f,  0.5f, -0.5f,  0.0f, 1.0f
		};


		unsigned int VBO, VAO;
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);


		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		// texture coord attribute
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);


		// load and create a texture 
		unsigned int texture1, texture2;

		// texture 1
		glGenTextures(1, &texture1);
		glBindTexture(GL_TEXTURE_2D, texture1);
		// set the texture wrapping parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		// set texture filtering parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		// load image, create texture and generate mipmaps
		int width, height, nrChannels;
		stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
		// The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
		unsigned char* data = stbi_load("../textura/container.jpg", &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
			glGenerateMipmap(GL_TEXTURE_2D);
		}
		else
		{
			std::cout << "Failed to load texture" << std::endl;
		}
		stbi_image_free(data);
		// texture 2
		glGenTextures(1, &texture2);
		glBindTexture(GL_TEXTURE_2D, texture2);
		// set the texture wrapping parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		// set texture filtering parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		// load image, create texture and generate mipmaps
		data = stbi_load("../textura/awesomeface.png", &width, &height, &nrChannels, 0);
		if (data)
		{
			// note that the awesomeface.png has transparency and thus an alpha channel, so make sure to tell OpenGL the data type is of GL_RGBA
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
			glGenerateMipmap(GL_TEXTURE_2D);
		}
		else
		{
			std::cout << "Failed to load texture" << std::endl;
		}
		stbi_image_free(data);

		// tell opengl for each sampler to which texture unit it belongs to (only has to be done once)
		ourShader.use();
		ourShader.setInt("texture1", 0);
		ourShader.setInt("texture2", 1);


		// render loop
		while (!glfwWindowShouldClose(window))
		{
			// per-frame time logi
			float currentFrame = static_cast<float>(glfwGetTime());
			deltaTime = currentFrame - lastFrame;
			lastFrame = currentFrame;


			// Atualiza a simulação física
			// Evita um passo muito grande caso a aplicação trave por alguns instantes.
			float physicsDeltaTime = std::min(deltaTime, 0.1f);

			physicsSystem.Update(
				physicsDeltaTime,
				1,
				&tempAllocator,
				&jobSystem
			);





			// input
			processInput(window);

			// render
			glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// bind textures on corresponding texture units
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1);
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2);

			// activate shader
			ourShader.use();

			// pass projection matrix to shader (note that in this case it could change every frame)
			glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
			ourShader.setMat4("projection", projection);

			// camera/view transformation
			glm::mat4 view = camera.GetViewMatrix();
			ourShader.setMat4("view", view);

			// render boxes
			glBindVertexArray(VAO);

			// -----------------------------------------
			// Desenha o chão
			// -----------------------------------------

			glm::mat4 floorModel = glm::mat4(1.0f);

			floorModel = glm::translate(
				floorModel,
				glm::vec3(0.0f, -0.25f, 0.0f)
			);

			floorModel = glm::scale(
				floorModel,
				glm::vec3(20.0f, 0.5f, 20.0f)
			);

			ourShader.setMat4("model", floorModel);

			glDrawArrays(GL_TRIANGLES, 0, 36);


			// -----------------------------------------
			// Desenha o cubo utilizando a posição do Jolt
			// -----------------------------------------

			JPH::RVec3 cubePosition =
				bodyInterface.GetPosition(cubeBody->GetID());

			glm::mat4 cubeModel = glm::mat4(1.0f);

			cubeModel = glm::translate(
				cubeModel,
				glm::vec3(
					(float)cubePosition.GetX(),
					(float)cubePosition.GetY(),
					(float)cubePosition.GetZ()
				)
			);

			ourShader.setMat4("model", cubeModel);

			glDrawArrays(GL_TRIANGLES, 0, 36);

			// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
			glfwSwapBuffers(window);
			glfwPollEvents();
		}

		// optional: de-allocate all resources once they've outlived their purpose:
		glDeleteVertexArrays(1, &VAO);
		glDeleteBuffers(1, &VBO);

		// -----------------------------------------
		// Finalização do Jolt Physics
		// -----------------------------------------
		bodyInterface.RemoveBody(cubeBody->GetID());
		bodyInterface.DestroyBody(cubeBody->GetID());

		bodyInterface.RemoveBody(floorBody->GetID());
		bodyInterface.DestroyBody(floorBody->GetID());
	}
	JPH::UnregisterTypes();

	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;

	// glfw: terminate, clearing all previously allocated GLFW resources.
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