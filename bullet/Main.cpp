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
#include <BulletCollision/Gimpact/btGImpactShape.h>            // malha dinâmica (GImpact)
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h> // algoritmo de colisão do GImpact

#include <iostream>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>


// =====================================================================
// CONFIGURAÇÃO DO TESTE (mude aqui e recompile)
// =====================================================================
// false = chão é uma caixa (colisão GImpact x forma convexa)
// true  = chão é uma MALHA ESTÁTICA (btBvhTriangleMeshShape):
//         testa GImpact (peça em movimento) x malha estática -> "malha com malha"
constexpr bool kFloorAsStaticMesh = false;

// true = cria uma segunda peça (girada) caindo sobre a primeira:
//        testa GImpact x GImpact -> "malha com malha", as duas em movimento
constexpr bool kSecondPart = true;

constexpr float kPartScale = 30.0f;     // a peça original tem ~0,17 m; com 30 fica com ~5 unidades
const btScalar  kBoxMass = 1.0f;        // massa de cada caixa da parede e do cubo
const btScalar  kPartMass = 1.0f;       // massa da peça (aproximada por uma caixa para a inércia)
const btScalar  kGImpactMargin = 0.02f; // margem de colisão do GImpact (ajustável)

// =====================================================================
// NOVO: TESTE DO FURO (tecla ESPAÇO)
// Solta um cubinho alinhado com o eixo de um furo da peça. Se a colisão fosse
// uma caixa em volta da peça, o cubo ficaria em cima; com a malha real ele
// atravessa o furo. Posições dos 8 furos passantes, no sistema LOCAL da peça
// (já escalada e centralizada), medidas direto da malha: { x, z }.
// =====================================================================
constexpr int kHoleCount = 8;
const float kHoleXZ[kHoleCount][2] = {
	{ 0.686f, -0.595f}, {-1.004f,  0.195f}, { 2.156f,  0.355f}, {-0.224f,  1.895f},
	{-2.254f, -1.905f}, { 2.306f, -1.445f}, {-2.254f,  0.945f}, { 1.476f,  1.105f}
};
constexpr float kHoleRadius = 0.155f;   // raio dos furos
constexpr float kDropHalf = 0.075f;     // meia-aresta do cubo (cabe no furo com folga)
constexpr float kDropHeight = 2.5f;     // distância acima da face da peça onde o cubo nasce
constexpr float kDropSpeed = 6.0f;      // velocidade inicial na direção do furo
bool gDropRequested = false;            // ligado pela tecla ESPAÇO


// =====================================================================
// NOVO: modos de visualização (alternados pelo teclado)
//   B = alterna entre MALHA (visual) e CORPO DE COLISÃO (Bullet)
//   O = liga/desliga a SOBREPOSIÇÃO (mostra os dois ao mesmo tempo)
// =====================================================================
bool gCollisionView = false;   // false = malha visual, true = corpo de colisão
bool gOverlay = false;         // true = desenha os dois juntos

static void OnViewModeChanged(GLFWwindow* window);


// Forward declarations
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);


// Window settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;


// Camera (afastada para enxergar a peça e a parede)
Camera camera(glm::vec3(0.0f, 6.0f, 20.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;


// Timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;


// =====================================================================
// Malha de triângulos compartilhada entre OpenGL e Bullet
// =====================================================================
// A MESMA geometria alimenta o desenho (OpenGL) e a colisão (Bullet).
struct MeshData
{
	std::vector<float> vertices;        // 5 floats por vértice: x y z u v
	std::vector<unsigned int> indices;  // 3 índices por triângulo
};

// Carrega o arquivo .mesh exportado do hier.lnc.
// Formato: uint32 nVerts, uint32 nIndices, float[nVerts*5] (x y z u v), uint32[nIndices]
static bool LoadMeshFile(const char* path, MeshData& out, float scale)
{
	std::ifstream f(path, std::ios::binary);
	if (!f)
		return false;

	uint32_t nV = 0, nI = 0;
	f.read(reinterpret_cast<char*>(&nV), 4);
	f.read(reinterpret_cast<char*>(&nI), 4);
	if (!f)
		return false;

	out.vertices.resize((size_t)nV * 5);
	out.indices.resize(nI);

	f.read(reinterpret_cast<char*>(out.vertices.data()),
		(std::streamsize)(out.vertices.size() * sizeof(float)));
	f.read(reinterpret_cast<char*>(out.indices.data()),
		(std::streamsize)(out.indices.size() * sizeof(uint32_t)));
	if (!f)
		return false;

	// aplica a escala em x y z (u v ficam como estão)
	for (size_t i = 0; i < out.vertices.size(); i += 5)
	{
		out.vertices[i] *= scale;
		out.vertices[i + 1] *= scale;
		out.vertices[i + 2] *= scale;
	}
	return true;
}

// Tenta abrir o .mesh em alguns lugares comuns
static bool LoadMeshFromKnownPaths(const char* fileName, MeshData& out, float scale)
{
	const char* folders[] = { "../textura/", "textura/", "", "../" };
	for (const char* folder : folders)
	{
		std::string path = std::string(folder) + fileName;
		if (LoadMeshFile(path.c_str(), out, scale))
		{
			std::cout << "Malha carregada: " << path << std::endl;
			return true;
		}
	}
	return false;
}

// Move a malha para que o centro da caixa envolvente fique na origem.
// No Bullet, a origem do corpo é o centro de massa assumido; peças de CAD costumam
// ter a origem deslocada, o que faria a peça girar de forma estranha.
// Devolve as dimensões (largura, altura, profundidade) da malha.
static glm::vec3 CenterMeshAtOrigin(MeshData& mesh)
{
	glm::vec3 mn(1e30f), mx(-1e30f);
	for (size_t i = 0; i + 4 < mesh.vertices.size(); i += 5)
	{
		glm::vec3 p(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]);
		mn = glm::min(mn, p);
		mx = glm::max(mx, p);
	}

	glm::vec3 center = (mn + mx) * 0.5f;
	for (size_t i = 0; i + 4 < mesh.vertices.size(); i += 5)
	{
		mesh.vertices[i] -= center.x;
		mesh.vertices[i + 1] -= center.y;
		mesh.vertices[i + 2] -= center.z;
	}
	return mx - mn;
}


// =====================================================================
// Bullet: armazenamento e criação de corpos
// =====================================================================

// O btTriangleIndexVertexArray NÃO copia os dados: os vetores abaixo precisam
// continuar vivos enquanto a forma GImpact existir.
struct BulletMeshStorage
{
	std::vector<btScalar> positions;   // 3 por vértice (btScalar pode ser float ou double)
	std::vector<int> indices;          // 3 por triângulo
	btTriangleIndexVertexArray* meshInterface = nullptr;
};

// Guarda tudo o que foi criado, para apagar na ordem certa no final.
struct PhysicsStore
{
	std::vector<btRigidBody*> bodies;
	std::vector<btDefaultMotionState*> motionStates;
	std::vector<btCollisionShape*> shapes;
	std::vector<BulletMeshStorage*> meshStorages;
	std::vector<btTriangleMesh*> triangleMeshes;
};

static btRigidBody* AddBody(btDiscreteDynamicsWorld* world, PhysicsStore& store,
	btCollisionShape* shape, btScalar mass, const btVector3& inertia,
	const btVector3& position, const btQuaternion& rotation)
{
	btTransform t;
	t.setIdentity();
	t.setOrigin(position);
	t.setRotation(rotation);

	btDefaultMotionState* motionState = new btDefaultMotionState(t);

	btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, shape, inertia);
	ci.m_friction = 0.5f;

	btRigidBody* body = new btRigidBody(ci);
	world->addRigidBody(body);

	store.motionStates.push_back(motionState);
	store.bodies.push_back(body);
	return body;
}

// Caixa dinâmica simples (convexa)
static btRigidBody* AddBox(btDiscreteDynamicsWorld* world, PhysicsStore& store,
	btCollisionShape* boxShape, btScalar mass, const btVector3& position)
{
	btVector3 inertia(0, 0, 0);
	boxShape->calculateLocalInertia(mass, inertia);
	return AddBody(world, store, boxShape, mass, inertia, position,
		btQuaternion(0, 0, 0, 1));
}

// Peça complexa como MALHA DINÂMICA (GImpact)
static btRigidBody* AddGImpactPart(btDiscreteDynamicsWorld* world, PhysicsStore& store,
	const MeshData& mesh, const glm::vec3& extents, btScalar mass,
	const btVector3& position, const btQuaternion& rotation)
{
	BulletMeshStorage* storage = new BulletMeshStorage();

	// copia as posições (descarta u v) para um vetor de btScalar
	for (size_t i = 0; i + 4 < mesh.vertices.size(); i += 5)
	{
		storage->positions.push_back((btScalar)mesh.vertices[i]);
		storage->positions.push_back((btScalar)mesh.vertices[i + 1]);
		storage->positions.push_back((btScalar)mesh.vertices[i + 2]);
	}
	for (unsigned int idx : mesh.indices)
		storage->indices.push_back((int)idx);

	storage->meshInterface = new btTriangleIndexVertexArray(
		(int)(storage->indices.size() / 3),
		storage->indices.data(),
		3 * (int)sizeof(int),
		(int)(storage->positions.size() / 3),
		storage->positions.data(),
		3 * (int)sizeof(btScalar));

	btGImpactMeshShape* shape = new btGImpactMeshShape(storage->meshInterface);
	shape->setMargin(kGImpactMargin);
	shape->updateBound();   // obrigatório depois de configurar a forma

	// Inércia: aproximada por uma caixa com as dimensões reais da malha
	// (mesma ideia do teste na Jolt, para a comparação ser justa).
	btBoxShape inertiaBox(btVector3(extents.x * 0.5f, extents.y * 0.5f, extents.z * 0.5f));
	btVector3 inertia(0, 0, 0);
	inertiaBox.calculateLocalInertia(mass, inertia);

	store.meshStorages.push_back(storage);
	store.shapes.push_back(shape);

	return AddBody(world, store, shape, mass, inertia, position, rotation);
}

// Caixa feita de triângulos (12), usada como chão em MALHA ESTÁTICA
static btBvhTriangleMeshShape* CreateStaticBoxMesh(PhysicsStore& store, const btVector3& h)
{
	btTriangleMesh* tm = new btTriangleMesh();

	const btVector3 v[8] = {
		btVector3(-h.x(), -h.y(), -h.z()), btVector3(h.x(), -h.y(), -h.z()),
		btVector3(h.x(),  h.y(), -h.z()), btVector3(-h.x(),  h.y(), -h.z()),
		btVector3(-h.x(), -h.y(),  h.z()), btVector3(h.x(), -h.y(),  h.z()),
		btVector3(h.x(),  h.y(),  h.z()), btVector3(-h.x(),  h.y(),  h.z())
	};

	const int tris[12][3] = {
		{0,2,1}, {0,3,2},   // face -z
		{4,5,6}, {4,6,7},   // face +z
		{0,1,5}, {0,5,4},   // face -y
		{3,7,6}, {3,6,2},   // face +y
		{0,4,7}, {0,7,3},   // face -x
		{1,2,6}, {1,6,5}    // face +x
	};
	for (const auto& t : tris)
		tm->addTriangle(v[t[0]], v[t[1]], v[t[2]]);

	store.triangleMeshes.push_back(tm);

	btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(tm, true);
	store.shapes.push_back(shape);
	return shape;
}

// Monta a matriz de modelo do OpenGL a partir da posição E rotação de um corpo do Bullet
static glm::mat4 BodyModel(btRigidBody* body)
{
	btTransform t;
	body->getMotionState()->getWorldTransform(t);

	const btVector3 p = t.getOrigin();
	const btQuaternion q = t.getRotation();

	glm::mat4 model = glm::translate(
		glm::mat4(1.0f),
		glm::vec3((float)p.getX(), (float)p.getY(), (float)p.getZ()));

	// GLM: construtor de quaternion usa a ordem w, x, y, z
	model *= glm::mat4_cast(glm::quat((float)q.getW(), (float)q.getX(), (float)q.getY(), (float)q.getZ()));
	return model;
}


// =====================================================================
// NOVO: shader simples de cor sólida, para desenhar o corpo de colisão
// (fica dentro deste arquivo: não depende dos seus arquivos .glsl)
// =====================================================================
static GLuint CompileGLShader(GLenum type, const char* source)
{
	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &source, nullptr);
	glCompileShader(shader);

	GLint ok = 0;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
	if (!ok)
	{
		char log[1024];
		glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
		std::cout << "Erro ao compilar shader de colisao: " << log << std::endl;
	}
	return shader;
}

static GLuint CreateFlatColorProgram()
{
	const char* vs =
		"#version 330 core\n"
		"layout (location = 0) in vec3 aPos;\n"
		"uniform mat4 model;\n"
		"uniform mat4 view;\n"
		"uniform mat4 projection;\n"
		"void main() { gl_Position = projection * view * model * vec4(aPos, 1.0); }\n";

	const char* fs =
		"#version 330 core\n"
		"out vec4 FragColor;\n"
		"uniform vec3 color;\n"
		"void main() { FragColor = vec4(color, 1.0); }\n";

	GLuint v = CompileGLShader(GL_VERTEX_SHADER, vs);
	GLuint f = CompileGLShader(GL_FRAGMENT_SHADER, fs);

	GLuint program = glCreateProgram();
	glAttachShader(program, v);
	glAttachShader(program, f);
	glLinkProgram(program);

	GLint ok = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &ok);
	if (!ok)
	{
		char log[1024];
		glGetProgramInfoLog(program, sizeof(log), nullptr, log);
		std::cout << "Erro ao linkar shader de colisao: " << log << std::endl;
	}

	glDeleteShader(v);
	glDeleteShader(f);
	return program;
}


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
		SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL - Bullet Physics 2.89 (GImpact)", nullptr, nullptr);

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
	btDefaultCollisionConfiguration* collisionConfiguration =
		new btDefaultCollisionConfiguration();

	btCollisionDispatcher* dispatcher =
		new btCollisionDispatcher(collisionConfiguration);

	// Registra o algoritmo de colisão do GImpact.
	// SEM ESTA LINHA a malha GImpact não colide com nada.
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	btSequentialImpulseConstraintSolver* solver =
		new btSequentialImpulseConstraintSolver();

	btDiscreteDynamicsWorld* dynamicsWorld =
		new btDiscreteDynamicsWorld(
			dispatcher,
			overlappingPairCache,
			solver,
			collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0.0f, -9.81f, 0.0f));

	PhysicsStore store;

	// ---------------------------------------------------------
	// Create physics objects
	// ---------------------------------------------------------

	// CHÃO: estático (massa 0). Caixa convexa OU malha estática, conforme a configuração.
	btRigidBody* floorBody = nullptr;
	{
		btCollisionShape* floorShape = nullptr;
		if (kFloorAsStaticMesh)
		{
			floorShape = CreateStaticBoxMesh(store, btVector3(10.0f, 0.25f, 10.0f));
			std::cout << "Chao: MALHA ESTATICA (btBvhTriangleMeshShape)" << std::endl;
		}
		else
		{
			floorShape = new btBoxShape(btVector3(10.0f, 0.25f, 10.0f));
			store.shapes.push_back(floorShape);
			std::cout << "Chao: caixa (btBoxShape)" << std::endl;
		}

		floorBody = AddBody(dynamicsWorld, store, floorShape, 0.0f, btVector3(0, 0, 0),
			btVector3(0.0f, -0.25f, 0.0f), btQuaternion(0, 0, 0, 1));
	}

	// CUBO: dinâmico (movido para o lado, para não nascer dentro da parede)
	btCollisionShape* cubeShape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
	store.shapes.push_back(cubeShape);
	btRigidBody* cubeBody = AddBox(dynamicsWorld, store, cubeShape, kBoxMass, btVector3(8.0f, 5.0f, 0.0f));
	cubeBody->setRestitution(0.1f);

	// PAREDE DE CAIXAS (dinâmicas), como no DynamicMeshTest da Jolt
	btCollisionShape* wallBoxShape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
	store.shapes.push_back(wallBoxShape);
	std::vector<btRigidBody*> wallBodies;

	for (int i = 0; i < 7; ++i)
		for (int j = i / 2; j < 7 - (i + 1) / 2; ++j)
		{
			btVector3 position(
				-3.5f + j * 1.0f + ((i & 1) ? 0.5f : 0.0f),
				0.5f + i * 1.0f,
				0.0f);

			wallBodies.push_back(AddBox(dynamicsWorld, store, wallBoxShape, kBoxMass, position));
		}

	// PEÇA COMPLEXA COMO MALHA DINÂMICA (hier_part0.mesh, exportada do hier.lnc)
	MeshData part;
	if (!LoadMeshFromKnownPaths("hier_part0.mesh", part, kPartScale))
	{
		std::cout << "Nao encontrou hier_part0.mesh (procurei em ../textura/, textura/, ./ e ../)" << std::endl;
		glfwTerminate();
		return -1;
	}

	const glm::vec3 partExtents = CenterMeshAtOrigin(part);
	std::cout << "Vertices: " << part.vertices.size() / 5
		<< "  Triangulos: " << part.indices.size() / 3 << std::endl;

	std::vector<btRigidBody*> partBodies;

	auto tBuild0 = std::chrono::high_resolution_clock::now();

	partBodies.push_back(AddGImpactPart(dynamicsWorld, store, part, partExtents, kPartMass,
		btVector3(0.0f, 10.0f, 0.0f), btQuaternion(0, 0, 0, 1)));

	if (kSecondPart)
	{
		// segunda peça, girada 90 graus, caindo sobre a primeira (GImpact x GImpact)
		partBodies.push_back(AddGImpactPart(dynamicsWorld, store, part, partExtents, kPartMass,
			btVector3(1.0f, 18.0f, 1.0f), btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI)));
		std::cout << "Segunda peca criada (teste GImpact x GImpact)" << std::endl;
	}

	// NOVO: forma e lista dos cubinhos do teste do furo
	btCollisionShape* dropShape = new btBoxShape(btVector3(kDropHalf, kDropHalf, kDropHalf));
	store.shapes.push_back(dropShape);
	std::vector<btRigidBody*> dropBodies;
	int nextHole = 0;

	auto tBuild1 = std::chrono::high_resolution_clock::now();
	std::cout << "Tempo para criar as formas GImpact: "
		<< std::chrono::duration<double, std::milli>(tBuild1 - tBuild0).count() << " ms" << std::endl;

	// ---------------------------------------------------------
	// OpenGL shader
	// ---------------------------------------------------------
	Shader ourShader("Vcolor.glsl", "Fcolor.glsl");

	// NOVO: shader de cor sólida para o modo de colisão
	GLuint flatProgram = CreateFlatColorProgram();
	const GLint flatModelLoc = glGetUniformLocation(flatProgram, "model");
	const GLint flatViewLoc = glGetUniformLocation(flatProgram, "view");
	const GLint flatProjLoc = glGetUniformLocation(flatProgram, "projection");
	const GLint flatColorLoc = glGetUniformLocation(flatProgram, "color");

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

	// NOVO: caixa só com as 12 arestas (GL_LINES), para desenhar as caixas de colisão
	float lineCube[] = {
		// base (y = -0.5)
		-0.5f,-0.5f,-0.5f,   0.5f,-0.5f,-0.5f,
		 0.5f,-0.5f,-0.5f,   0.5f,-0.5f, 0.5f,
		 0.5f,-0.5f, 0.5f,  -0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,  -0.5f,-0.5f,-0.5f,
		// topo (y = +0.5)
		-0.5f, 0.5f,-0.5f,   0.5f, 0.5f,-0.5f,
		 0.5f, 0.5f,-0.5f,   0.5f, 0.5f, 0.5f,
		 0.5f, 0.5f, 0.5f,  -0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,  -0.5f, 0.5f,-0.5f,
		// arestas verticais
		-0.5f,-0.5f,-0.5f,  -0.5f, 0.5f,-0.5f,
		 0.5f,-0.5f,-0.5f,   0.5f, 0.5f,-0.5f,
		 0.5f,-0.5f, 0.5f,   0.5f, 0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,  -0.5f, 0.5f, 0.5f
	};

	unsigned int lineCubeVAO, lineCubeVBO;
	glGenVertexArrays(1, &lineCubeVAO);
	glGenBuffers(1, &lineCubeVBO);
	glBindVertexArray(lineCubeVAO);
	glBindBuffer(GL_ARRAY_BUFFER, lineCubeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(lineCube), lineCube, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	// VAO/VBO/EBO da peça (mesmo layout: posição + coordenada de textura)
	unsigned int partVAO, partVBO, partEBO;
	glGenVertexArrays(1, &partVAO);
	glGenBuffers(1, &partVBO);
	glGenBuffers(1, &partEBO);

	glBindVertexArray(partVAO);

	glBindBuffer(GL_ARRAY_BUFFER, partVBO);
	glBufferData(GL_ARRAY_BUFFER,
		part.vertices.size() * sizeof(float),
		part.vertices.data(), GL_STATIC_DRAW);

	// o EBO precisa ser ligado enquanto o VAO da peça está ativo
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, partEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		part.indices.size() * sizeof(unsigned int),
		part.indices.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0);

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


	// medição do tempo gasto pela física (média por segundo), para comparar com a Jolt
	double physicsMsAccum = 0.0;
	int physicsFrames = 0;
	float lastReport = 0.0f;

	// NOVO: mostra a legenda do modo inicial
	OnViewModeChanged(window);


	// ---------------------------------------------------------
	// Render loop
	// ---------------------------------------------------------
	while (!glfwWindowShouldClose(window))
	{
		const float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// NOVO: ESPAÇO -> solta um cubo sobre um furo da peça (alinhado com o eixo do furo)
		if (gDropRequested)
		{
			gDropRequested = false;

			btRigidBody* target = partBodies[0];
			const glm::mat4 pm = BodyModel(target);

			// lado da peça que está mais para cima no mundo (o furo é passante nos dois sentidos)
			const float sign = (pm * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)).y >= 0.0f ? 1.0f : -1.0f;

			const float* h = kHoleXZ[nextHole];
			nextHole = (nextHole + 1) % kHoleCount;

			const glm::vec3 localSpawn(h[0], sign * (partExtents.y * 0.5f + kDropHeight), h[1]);
			const glm::vec3 worldSpawn = glm::vec3(pm * glm::vec4(localSpawn, 1.0f));
			const glm::vec3 dir = glm::normalize(glm::vec3(pm * glm::vec4(0.0f, -sign, 0.0f, 0.0f)));

			btTransform partT;
			target->getMotionState()->getWorldTransform(partT);

			const btScalar dropMass = 0.2f;
			btVector3 dropInertia(0, 0, 0);
			dropShape->calculateLocalInertia(dropMass, dropInertia);

			// mesma rotação da peça: o cubo desce alinhado com o furo
			btRigidBody* c = AddBody(dynamicsWorld, store, dropShape, dropMass, dropInertia,
				btVector3(worldSpawn.x, worldSpawn.y, worldSpawn.z), partT.getRotation());
			c->setLinearVelocity(btVector3(dir.x, dir.y, dir.z) * kDropSpeed);
			dropBodies.push_back(c);

			std::cout << "Cubo solto sobre o furo " << (nextHole + kHoleCount - 1) % kHoleCount + 1
				<< " de " << kHoleCount << " (raio do furo " << kHoleRadius
				<< ", cubo de " << kDropHalf * 2.0f << ")" << std::endl;
		}

		// Evita um passo enorme depois de uma pausa/travada.
		const btScalar physicsDeltaTime =
			static_cast<btScalar>(std::min(deltaTime, 0.1f));

		// O Bullet usa um passo interno fixo (1/60 s) e pode fazer até 10 subpassos.
		auto p0 = std::chrono::high_resolution_clock::now();
		dynamicsWorld->stepSimulation(physicsDeltaTime, 10, 1.0f / 60.0f);
		auto p1 = std::chrono::high_resolution_clock::now();

		physicsMsAccum += std::chrono::duration<double, std::milli>(p1 - p0).count();
		++physicsFrames;
		if (currentFrame - lastReport >= 1.0f)
		{
			std::cout << "Fisica: media " << physicsMsAccum / physicsFrames
				<< " ms por frame (" << physicsFrames << " frames)" << std::endl;
			physicsMsAccum = 0.0;
			physicsFrames = 0;
			lastReport = currentFrame;
		}

		processInput(window);

		// NOVO: o que desenhar neste frame
		const bool drawMesh = !gCollisionView || gOverlay;
		const bool drawCollision = gCollisionView || gOverlay;

		// no modo só-colisão o fundo escurece para destacar as linhas
		if (gCollisionView && !gOverlay)
			glClearColor(0.04f, 0.05f, 0.08f, 1.0f);
		else
			glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 projection = glm::perspective(
			glm::radians(camera.Zoom),
			static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT),
			0.1f, 100.0f);
		glm::mat4 view = camera.GetViewMatrix();

		// Chão: o cubo do OpenGL tem tamanho 1, então escala para 20 x 0,5 x 20
		glm::mat4 floorModel(1.0f);
		floorModel = glm::translate(floorModel, glm::vec3(0.0f, -0.25f, 0.0f));
		floorModel = glm::scale(floorModel, glm::vec3(20.0f, 0.5f, 20.0f));


		// =========================================================
		// MODO MALHA: o que o OpenGL desenha (malha visual com textura)
		// =========================================================
		if (drawMesh)
		{
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1);
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2);

			ourShader.use();
			ourShader.setMat4("projection", projection);
			ourShader.setMat4("view", view);

			glBindVertexArray(VAO);

			// chão
			ourShader.setMat4("model", floorModel);
			glDrawArrays(GL_TRIANGLES, 0, 36);

			// cubo (posição E rotação do Bullet)
			ourShader.setMat4("model", BodyModel(cubeBody));
			glDrawArrays(GL_TRIANGLES, 0, 36);

			// parede de caixas
			for (btRigidBody* wall : wallBodies)
			{
				ourShader.setMat4("model", BodyModel(wall));
				glDrawArrays(GL_TRIANGLES, 0, 36);
			}

			// NOVO: cubinhos soltos sobre os furos
			for (btRigidBody* c : dropBodies)
			{
				ourShader.setMat4("model", BodyModel(c) * glm::scale(glm::mat4(1.0f), glm::vec3(kDropHalf * 2.0f)));
				glDrawArrays(GL_TRIANGLES, 0, 36);
			}

			// peça(s) complexa(s) (malha dinâmica)
			glBindVertexArray(partVAO);
			for (btRigidBody* p : partBodies)
			{
				ourShader.setMat4("model", BodyModel(p));
				glDrawElements(GL_TRIANGLES, (GLsizei)part.indices.size(), GL_UNSIGNED_INT, 0);
			}
		}


		// =========================================================
		// MODO COLISÃO: o que o Bullet usa de verdade (formas de colisão)
		// =========================================================
		if (drawCollision)
		{
			glUseProgram(flatProgram);
			glUniformMatrix4fv(flatProjLoc, 1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(flatViewLoc, 1, GL_FALSE, glm::value_ptr(view));

			auto setModel = [&](const glm::mat4& m)
				{
					glUniformMatrix4fv(flatModelLoc, 1, GL_FALSE, glm::value_ptr(m));
				};
			auto setColor = [&](const glm::vec3& c)
				{
					glUniform3f(flatColorLoc, c.x, c.y, c.z);
				};
			// corpo adormecido (sleeping) = cor mais escura
			auto shade = [&](btRigidBody* body, const glm::vec3& c)
				{
					return body->isActive() ? c : c * 0.4f;
				};

			const glm::vec3 kBoxColor(0.2f, 1.0f, 0.3f);    // verde: caixa (convexa)
			const glm::vec3 kMeshColor(1.0f, 0.55f, 0.1f);  // laranja: malha de triângulos
			const glm::vec3 kMassColor(0.2f, 0.85f, 1.0f);  // ciano: caixa só de massa/inércia
			const glm::vec3 kFloorColor(0.6f, 0.65f, 0.75f);

			// pequena folga para as linhas não "brigarem" com as faces texturizadas
			const glm::mat4 kGrow = glm::scale(glm::mat4(1.0f), glm::vec3(1.004f));

			// --- caixas de colisão (arestas) ---
			glBindVertexArray(lineCubeVAO);

			// chão (estático): cinza se for caixa; laranja se for malha estática
			setModel(floorModel * kGrow);
			setColor(kFloorAsStaticMesh ? kMeshColor : kFloorColor);
			glDrawArrays(GL_LINES, 0, 24);

			// cubo
			setModel(BodyModel(cubeBody) * kGrow);
			setColor(shade(cubeBody, kBoxColor));
			glDrawArrays(GL_LINES, 0, 24);

			// parede
			for (btRigidBody* wall : wallBodies)
			{
				setModel(BodyModel(wall) * kGrow);
				setColor(shade(wall, kBoxColor));
				glDrawArrays(GL_LINES, 0, 24);
			}

			// NOVO: cubinhos soltos sobre os furos (também são caixas convexas)
			for (btRigidBody* c : dropBodies)
			{
				setModel(BodyModel(c) * glm::scale(glm::mat4(1.0f), glm::vec3(kDropHalf * 2.0f)));
				setColor(shade(c, kBoxColor));
				glDrawArrays(GL_LINES, 0, 24);
			}

			// caixa usada SÓ para massa e inércia da peça (não participa da colisão)
			for (btRigidBody* p : partBodies)
			{
				setModel(BodyModel(p) * glm::scale(glm::mat4(1.0f), partExtents));
				setColor(shade(p, kMassColor));
				glDrawArrays(GL_LINES, 0, 24);
			}

			// --- malha de triângulos que o Bullet (GImpact) usa na colisão da peça ---
			glBindVertexArray(partVAO);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glEnable(GL_POLYGON_OFFSET_LINE);
			glPolygonOffset(-1.0f, -1.0f);

			for (btRigidBody* p : partBodies)
			{
				setModel(BodyModel(p));
				setColor(shade(p, kMeshColor));
				glDrawElements(GL_TRIANGLES, (GLsizei)part.indices.size(), GL_UNSIGNED_INT, 0);
			}

			glDisable(GL_POLYGON_OFFSET_LINE);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// ---------------------------------------------------------
	// Cleanup OpenGL resources
	// ---------------------------------------------------------
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteVertexArrays(1, &partVAO);
	glDeleteBuffers(1, &partVBO);
	glDeleteBuffers(1, &partEBO);
	glDeleteVertexArrays(1, &lineCubeVAO);   // NOVO
	glDeleteBuffers(1, &lineCubeVBO);        // NOVO
	glDeleteProgram(flatProgram);            // NOVO
	glDeleteTextures(1, &texture1);
	glDeleteTextures(1, &texture2);

	// ---------------------------------------------------------
	// Cleanup Bullet objects
	// Ordem: corpos -> estados de movimento -> formas -> dados das malhas -> mundo
	// ---------------------------------------------------------
	for (btRigidBody* body : store.bodies)
	{
		dynamicsWorld->removeRigidBody(body);
		delete body;
	}
	for (btDefaultMotionState* ms : store.motionStates)
		delete ms;

	for (btCollisionShape* shape : store.shapes)
		delete shape;

	for (BulletMeshStorage* storage : store.meshStorages)
	{
		delete storage->meshInterface;
		delete storage;
	}
	for (btTriangleMesh* tm : store.triangleMeshes)
		delete tm;

	delete dynamicsWorld;
	delete solver;
	delete overlappingPairCache;
	delete dispatcher;
	delete collisionConfiguration;

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


// NOVO: chamado quando o modo de visualização muda (título da janela + legenda no console)
static void OnViewModeChanged(GLFWwindow* window)
{
	std::string title = "Bullet Physics | ";

	if (gOverlay)
		title += "SOBREPOSICAO: malha + corpo de colisao";
	else if (gCollisionView)
		title += "CORPO DE COLISAO (Bullet)";
	else
		title += "MALHA (visual)";

	title += "  [B = alterna, O = sobrepoe, ESPACO = cubo no furo]";
	glfwSetWindowTitle(window, title.c_str());

	std::cout << "\n=== " << title << " ===" << std::endl;
	if (gCollisionView || gOverlay)
	{
		std::cout << "  Verde   = caixas (btBoxShape, forma convexa) - colidem\n"
			<< "  Laranja = malha de triangulos da peca (btGImpactMeshShape) - colide";
		if (kFloorAsStaticMesh)
			std::cout << "\n            (o chao tambem e malha: btBvhTriangleMeshShape, 12 triangulos)";
		std::cout << "\n"
			<< "  Ciano   = caixa usada so para massa/inercia da peca - NAO colide\n";
		if (!kFloorAsStaticMesh)
			std::cout << "  Cinza   = chao (estatico)\n";
		std::cout << "  Cores escuras = corpo adormecido (sleeping)" << std::endl;
	}
	else
	{
		std::cout << "  Malha visual: o que o OpenGL desenha (texturas). A fisica nao a enxerga." << std::endl;
	}
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

	// NOVO: B alterna malha <-> colisão; O liga/desliga a sobreposição.
	// Só reage no momento em que a tecla é apertada (não repete enquanto segura).
	static bool bWasDown = false;
	static bool oWasDown = false;

	const bool bDown = glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS;
	if (bDown && !bWasDown)
	{
		gCollisionView = !gCollisionView;
		OnViewModeChanged(window);
	}
	bWasDown = bDown;

	const bool oDown = glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS;
	if (oDown && !oWasDown)
	{
		gOverlay = !gOverlay;
		OnViewModeChanged(window);
	}
	oWasDown = oDown;

	// NOVO: ESPAÇO solta um cubo sobre um furo (reage só ao apertar, não ao segurar)
	static bool spaceWasDown = false;
	const bool spaceDown = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
	if (spaceDown && !spaceWasDown)
		gDropRequested = true;
	spaceWasDown = spaceDown;
}


// glfw: whenever the window size changed (by OS or user resize) this callback function execute
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
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