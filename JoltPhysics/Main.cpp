#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "stb_image.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>   // converter rotação do Jolt para matriz

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
#include <Jolt/Physics/Collision/Shape/MeshShape.h>   // malha de triângulos
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>      // NOVO: casco convexo
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>  // NOVO: união de formas convexas
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#include <thread>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <utility>


// =====================================================================
// CONFIGURAÇÃO DO TESTE (mude aqui e recompile)
// =====================================================================
// Threads EXTRAS do Jolt. 0 = só a thread principal.
constexpr int kJoltWorkerThreads = 2;

// Move a malha para que o centro da caixa envolvente fique na origem.
constexpr bool kCenterMeshAtOrigin = true;

// =====================================================================
// NOVO: SEGUNDA PEÇA (teste de colisão entre dois objetos complexos)
// O Jolt NÃO colide malha com malha (MeshShape x MeshShape). Por isso a forma
// da segunda peça é escolhida aqui:
//   0 = MeshShape igual à primeira (o Jolt não suporta: as peças devem se atravessar;
//       em build Debug pode até disparar um assert)
//   1 = UM casco convexo da peça inteira (colide, mas perde os furos e as reentrâncias)
//   2 = COMPOUND de vários cascos convexos, feitos por decomposição em grade
//       (colide e acompanha melhor o contorno)
// =====================================================================
constexpr bool kSecondPart = true;
constexpr int kSecondPartMode = 2;
constexpr float kDecompCell = 0.5f;   // tamanho da célula da grade (menor = mais cascos, mais fiel)


// =====================================================================
// Modos de visualização (alternados pelo teclado)
//   B = alterna entre MALHA (visual) e CORPO DE COLISÃO (Jolt)
//   O = liga/desliga a SOBREPOSIÇÃO (mostra os dois ao mesmo tempo)
// =====================================================================
bool gCollisionView = false;   // false = malha visual, true = corpo de colisão
bool gOverlay = false;         // true = desenha os dois juntos

static void OnViewModeChanged(GLFWwindow* window);


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


// =====================================================================
// Malha de triângulos compartilhada entre OpenGL e Jolt
// =====================================================================
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

// Dimensões (largura, altura, profundidade) da malha
static glm::vec3 MeshExtents(const MeshData& mesh)
{
	glm::vec3 mn(1e30f), mx(-1e30f);
	for (size_t i = 0; i + 4 < mesh.vertices.size(); i += 5)
	{
		glm::vec3 p(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]);
		mn = glm::min(mn, p);
		mx = glm::max(mx, p);
	}
	return mx - mn;
}

// Move a malha para que o centro da caixa envolvente fique na origem
static void CenterMeshAtOrigin(MeshData& mesh)
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
}

// Converte MeshData em uma MeshShape do Jolt
static JPH::ShapeRefC CreateJoltMeshShape(const MeshData& mesh)
{
	JPH::VertexList vertices;
	for (size_t i = 0; i + 4 < mesh.vertices.size(); i += 5)
		vertices.push_back(JPH::Float3(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]));

	JPH::IndexedTriangleList triangles;

	for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
	{
		unsigned int a = mesh.indices[i];
		unsigned int b = mesh.indices[i + 1];
		unsigned int c = mesh.indices[i + 2];

		if (a >= mesh.vertices.size() / 5 ||
			b >= mesh.vertices.size() / 5 ||
			c >= mesh.vertices.size() / 5)
		{
			std::cout << "Indice invalido na malha!" << std::endl;
			return nullptr;
		}

		triangles.push_back(
			JPH::IndexedTriangle(a, b, c)
		);
	}

	JPH::MeshShapeSettings settings(vertices, triangles);
	settings.SetEmbedded();   // settings vive na pilha: evita o Jolt tentar apagá-la

	JPH::ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
	{
		std::cout << "Erro ao criar MeshShape: " << result.GetError().c_str() << std::endl;
		return nullptr;
	}
	return result.Get();
}

// =====================================================================
// NOVO: aproximações CONVEXAS da peça (o que o Jolt consegue colidir com uma MeshShape)
// =====================================================================

static std::vector<float> ExtractShapeTriangles(const JPH::Shape* shape);

// Casco convexo de um conjunto de pontos (nullptr se os pontos forem degenerados)
static JPH::ShapeRefC CreateHull(const JPH::Array<JPH::Vec3>& points)
{
	if (points.size() < 4)
		return nullptr;

	JPH::ConvexHullShapeSettings settings(points, 0.01f);
	settings.SetEmbedded();

	JPH::ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
		return nullptr;
	return result.Get();
}

// UM casco convexo com todos os vértices da peça
static JPH::ShapeRefC CreateSingleHull(const MeshData& mesh)
{
	JPH::Array<JPH::Vec3> points;
	for (size_t i = 0; i + 4 < mesh.vertices.size(); i += 5)
		points.push_back(JPH::Vec3(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]));

	JPH::ShapeRefC hull = CreateHull(points);
	if (hull == nullptr)
		std::cout << "Falha ao criar o casco convexo unico" << std::endl;
	return hull;
}

// Decomposição simples em grade (plano XZ): cada triângulo vai para a célula do seu centro,
// e cada célula vira um casco convexo. Os cascos se sobrepõem um pouco, o que não é problema.
// Não é uma decomposição convexa "de verdade" (V-HACD/CoACD, que não fazem parte do Jolt):
// os furos tendem a ser preenchidos.
static JPH::ShapeRefC CreateCompoundOfHulls(const MeshData& mesh, float cell, int& outHulls, int& outSkipped,
	std::vector<float>& outTriangles)
{
	std::map<std::pair<int, int>, JPH::Array<JPH::Vec3>> cells;

	auto P = [&](unsigned int v)
		{
			return JPH::Vec3(mesh.vertices[v * 5], mesh.vertices[v * 5 + 1], mesh.vertices[v * 5 + 2]);
		};

	for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
	{
		const JPH::Vec3 a = P(mesh.indices[i]);
		const JPH::Vec3 b = P(mesh.indices[i + 1]);
		const JPH::Vec3 c = P(mesh.indices[i + 2]);
		const JPH::Vec3 center = (a + b + c) * (1.0f / 3.0f);

		const std::pair<int, int> key(
			(int)std::floor(center.GetX() / cell),
			(int)std::floor(center.GetZ() / cell));

		JPH::Array<JPH::Vec3>& pts = cells[key];
		pts.push_back(a);
		pts.push_back(b);
		pts.push_back(c);
	}

	JPH::StaticCompoundShapeSettings compound;
	compound.SetEmbedded();

	outHulls = 0;
	outSkipped = 0;
	for (auto& kv : cells)
	{
		JPH::ShapeRefC hull = CreateHull(kv.second);
		if (hull == nullptr)
		{
			++outSkipped;   // célula degenerada (pontos coplanares ou poucos pontos)
			continue;
		}
		compound.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(), hull.GetPtr());
		++outHulls;

		// triângulos deste casco para o desenho (o casco fica na posição zero do compound,
		// então estas coordenadas já estão no sistema local da peça)
		std::vector<float> hullTris = ExtractShapeTriangles(hull.GetPtr());
		outTriangles.insert(outTriangles.end(), hullTris.begin(), hullTris.end());
	}

	if (outHulls == 0)
		return nullptr;

	JPH::ShapeSettings::ShapeResult result = compound.Create();
	if (result.HasError())
	{
		std::cout << "Erro ao criar o compound: " << result.GetError().c_str() << std::endl;
		return nullptr;
	}
	return result.Get();
}

// Extrai os triângulos de QUALQUER forma do Jolt (para desenhar o corpo de colisão real).
// Coordenadas no sistema local do corpo (mesmo sistema da malha visual).
static std::vector<float> ExtractShapeTriangles(const JPH::Shape* shape)
{
	std::vector<float> out;

	// caixa grande, mas FINITA (sBiggest pode estourar em algumas formas e não devolver nada)
	const JPH::AABox everything(JPH::Vec3::sReplicate(-1000.0f), JPH::Vec3::sReplicate(1000.0f));

	JPH::Shape::GetTrianglesContext context;
	shape->GetTrianglesStart(context, everything, shape->GetCenterOfMass(),
		JPH::Quat::sIdentity(), JPH::Vec3::sReplicate(1.0f));

	JPH::Float3 verts[JPH::Shape::cGetTrianglesMinTrianglesRequested * 3];
	for (;;)
	{
		const int n = shape->GetTrianglesNext(context, JPH::Shape::cGetTrianglesMinTrianglesRequested, verts);
		if (n == 0)
			break;
		for (int i = 0; i < n * 3; ++i)
		{
			out.push_back(verts[i].x);
			out.push_back(verts[i].y);
			out.push_back(verts[i].z);
		}
	}
	return out;
}


// Monta a matriz de modelo do OpenGL a partir da posição E rotação de um corpo do Jolt
static glm::mat4 BodyModel(JPH::BodyInterface& bodyInterface, JPH::BodyID id)
{
	JPH::RVec3 p = bodyInterface.GetPosition(id);
	JPH::Quat q = bodyInterface.GetRotation(id);

	glm::mat4 model = glm::translate(
		glm::mat4(1.0f),
		glm::vec3((float)p.GetX(), (float)p.GetY(), (float)p.GetZ()));

	model *= glm::mat4_cast(glm::quat(q.GetW(), q.GetX(), q.GetY(), q.GetZ()));
	return model;
}


// =====================================================================
// Shader simples de cor sólida, para desenhar o corpo de colisão
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



void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera (afastada para enxergar a peça e a parede)
Camera camera(glm::vec3(0.0f, 6.0f, 20.0f));
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
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL - Jolt Physics", NULL, NULL);
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
			kJoltWorkerThreads
		);
		std::cout << "Jolt: " << kJoltWorkerThreads << " thread(s) extra(s) alem da principal" << std::endl;

		// Interface para criar e manipular corpos físicos
		JPH::BodyInterface& bodyInterface =
			physicsSystem.GetBodyInterface();



		// -----------------------------------------
		// Criação dos objetos físicos
		// -----------------------------------------

		// CHÃO: objeto estático (caixa = forma convexa)
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

		JPH::BodyCreationSettings cubeSettings(
			cubeShape,
			JPH::RVec3(8.0, 5.0, 0.0),
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


		// PAREDE DE CAIXAS (dinâmicas), como no DynamicMeshTest
		JPH::ShapeRefC wallBoxShape = new JPH::BoxShape(JPH::Vec3::sReplicate(0.5f));
		std::vector<JPH::BodyID> wallIDs;

		for (int i = 0; i < 7; ++i)
			for (int j = i / 2; j < 7 - (i + 1) / 2; ++j)
			{
				JPH::RVec3 position(
					-3.5f + j * 1.0f + ((i & 1) ? 0.5f : 0.0f),
					0.5f + i * 1.0f,
					0.0f);

				wallIDs.push_back(bodyInterface.CreateAndAddBody(
					JPH::BodyCreationSettings(
						wallBoxShape,
						position,
						JPH::Quat::sIdentity(),
						JPH::EMotionType::Dynamic,
						Layers::MOVING),
					JPH::EActivation::Activate));
			}


		// PEÇA COMPLEXA COMO MALHA DINÂMICA (hier_part0.mesh)
		const float partScale = 30.0f;
		MeshData part;
		if (!LoadMeshFromKnownPaths("hier_part0.mesh", part, partScale))
		{
			std::cout << "Nao encontrou hier_part0.mesh (procurei em ../textura/, textura/, ./ e ../)" << std::endl;
			glfwTerminate();
			return -1;
		}

		if (kCenterMeshAtOrigin)
			CenterMeshAtOrigin(part);

		std::cout << "Vertices: " << part.vertices.size() / 5
			<< "  Triangulos: " << part.indices.size() / 3 << std::endl;

		auto tBuild0 = std::chrono::high_resolution_clock::now();
		JPH::ShapeRefC partShape = CreateJoltMeshShape(part);
		auto tBuild1 = std::chrono::high_resolution_clock::now();

		if (partShape == nullptr)
		{
			glfwTerminate();
			return -1;
		}
		std::cout << "Tempo para criar a forma MeshShape: "
			<< std::chrono::duration<double, std::milli>(tBuild1 - tBuild0).count() << " ms" << std::endl;

		JPH::BodyCreationSettings partSettings(
			partShape,
			JPH::RVec3(0.0, 10.0, 0.0),
			JPH::Quat::sIdentity(),
			JPH::EMotionType::Dynamic,
			Layers::MOVING
		);

		// A malha NÃO calcula massa nem inércia: precisamos informar.
		// Aproximamos a peça por uma caixa sólida com as dimensões reais da malha.
		glm::vec3 ext = MeshExtents(part);
		partSettings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
		partSettings.mMassPropertiesOverride.SetMassAndInertiaOfSolidBox(
			JPH::Vec3(ext.x, ext.y, ext.z), 10.0f);
		partSettings.mFriction = 0.5f;

		JPH::BodyID partID = bodyInterface.CreateAndAddBody(
			partSettings, JPH::EActivation::Activate);

		// NOVO: SEGUNDA PEÇA (colisão entre dois objetos complexos)
		JPH::BodyID part2ID;
		std::vector<float> part2Tris;   // triângulos da forma de colisão da peça 2 (para desenhar)
		if (kSecondPart)
		{
			JPH::ShapeRefC part2Shape;

			if (kSecondPartMode == 0)
			{
				part2Shape = partShape;   // mesma MeshShape: Jolt não colide malha x malha
				std::cout << "Peca 2: MeshShape (ATENCAO: o Jolt nao suporta malha x malha)" << std::endl;
			}
			else if (kSecondPartMode == 1)
			{
				auto t0 = std::chrono::high_resolution_clock::now();
				part2Shape = CreateSingleHull(part);
				auto t1 = std::chrono::high_resolution_clock::now();
				std::cout << "Peca 2: UM casco convexo ("
					<< std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms)" << std::endl;
			}
			else
			{
				int hulls = 0, skipped = 0;
				auto t0 = std::chrono::high_resolution_clock::now();
				part2Shape = CreateCompoundOfHulls(part, kDecompCell, hulls, skipped, part2Tris);
				auto t1 = std::chrono::high_resolution_clock::now();
				std::cout << "Peca 2: compound de " << hulls << " cascos convexos (celula " << kDecompCell
					<< ", " << skipped << " celulas descartadas) em "
					<< std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms" << std::endl;
			}

			if (part2Shape == nullptr)
			{
				std::cout << "Nao foi possivel criar a forma da peca 2" << std::endl;
				glfwTerminate();
				return -1;
			}

			// segunda peça girada 90 graus, caindo sobre a primeira (igual ao teste do Bullet)
			JPH::BodyCreationSettings part2Settings(
				part2Shape,
				JPH::RVec3(1.0, 18.0, 1.0),
				JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f * JPH::JPH_PI),
				JPH::EMotionType::Dynamic,
				Layers::MOVING
			);
			part2Settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
			part2Settings.mMassPropertiesOverride.SetMassAndInertiaOfSolidBox(
				JPH::Vec3(ext.x, ext.y, ext.z), 10.0f);
			part2Settings.mFriction = 0.5f;

			part2ID = bodyInterface.CreateAndAddBody(part2Settings, JPH::EActivation::Activate);

			// triângulos da forma de colisão (só para desenhar; no modo 0 usa a própria malha)
			// (no modo 2 os triângulos já foram coletados casco a casco na criação do compound)
			if (kSecondPartMode == 1)
				part2Tris = ExtractShapeTriangles(part2Shape);

			if (kSecondPartMode != 0)
				std::cout << "Peca 2: " << part2Tris.size() / 9 << " triangulos de colisao para desenhar" << std::endl;
		}

		// NOVO: forma e lista dos cubinhos do teste do furo
		JPH::ShapeRefC dropShape = new JPH::BoxShape(JPH::Vec3::sReplicate(kDropHalf));
		std::vector<JPH::BodyID> dropIDs;
		int nextHole = 0;

		// Recomendado depois de adicionar muitos corpos de uma vez
		physicsSystem.OptimizeBroadPhase();



		// build and compile our shader zprogram
		Shader ourShader("Vcolor.glsl", "Fcolor.glsl");

		// shader de cor sólida para o modo de colisão
		GLuint flatProgram = CreateFlatColorProgram();
		const GLint flatModelLoc = glGetUniformLocation(flatProgram, "model");
		const GLint flatViewLoc = glGetUniformLocation(flatProgram, "view");
		const GLint flatProjLoc = glGetUniformLocation(flatProgram, "projection");
		const GLint flatColorLoc = glGetUniformLocation(flatProgram, "color");

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


		// caixa só com as 12 arestas (GL_LINES), para desenhar as caixas de colisão
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


		// NOVO: VAO dos triângulos da forma de colisão da peça 2 (só posição, sem índices)
		unsigned int part2VAO = 0, part2VBO = 0;
		GLsizei part2VertexCount = 0;
		if (!part2Tris.empty())
		{
			glGenVertexArrays(1, &part2VAO);
			glGenBuffers(1, &part2VBO);
			glBindVertexArray(part2VAO);
			glBindBuffer(GL_ARRAY_BUFFER, part2VBO);
			glBufferData(GL_ARRAY_BUFFER, part2Tris.size() * sizeof(float), part2Tris.data(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);
			glBindVertexArray(0);
			part2VertexCount = (GLsizei)(part2Tris.size() / 3);
		}

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


		// load and create a texture 
		unsigned int texture1, texture2;

		// texture 1
		glGenTextures(1, &texture1);
		glBindTexture(GL_TEXTURE_2D, texture1);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		int width, height, nrChannels;
		stbi_set_flip_vertically_on_load(true);
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
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		data = stbi_load("../textura/awesomeface.png", &width, &height, &nrChannels, 0);
		if (data)
		{
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


		// passo fixo de física (60 Hz) com acumulador
		float physicsAccumulator = 0.0f;
		const float physicsStep = 1.0f / 60.0f;

		// medição (mesmas métricas do teste Bullet)
		double physicsMsTotal = 0.0;
		double physicsMsWorstFrame = 0.0;
		int physicsFrames = 0;
		int physicsSteps = 0;
		float lastReport = 0.0f;

		// mostra a legenda do modo inicial
		OnViewModeChanged(window);


		// render loop
		while (!glfwWindowShouldClose(window))
		{
			// per-frame time logi
			float currentFrame = static_cast<float>(glfwGetTime());
			deltaTime = currentFrame - lastFrame;
			lastFrame = currentFrame;


			// NOVO: ESPAÇO -> solta um cubo sobre um furo da peça (alinhado com o eixo do furo)
			if (gDropRequested)
			{
				gDropRequested = false;

				const glm::mat4 pm = BodyModel(bodyInterface, partID);

				// lado da peça que está mais para cima no mundo (o furo é passante nos dois sentidos)
				const float sign = (pm * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)).y >= 0.0f ? 1.0f : -1.0f;

				const float* h = kHoleXZ[nextHole];
				nextHole = (nextHole + 1) % kHoleCount;

				const glm::vec3 localSpawn(h[0], sign * (ext.y * 0.5f + kDropHeight), h[1]);
				const glm::vec3 worldSpawn = glm::vec3(pm * glm::vec4(localSpawn, 1.0f));
				const glm::vec3 dir = glm::normalize(glm::vec3(pm * glm::vec4(0.0f, -sign, 0.0f, 0.0f)));

				// mesma rotação da peça: o cubo desce alinhado com o furo
				JPH::BodyCreationSettings dropSettings(
					dropShape,
					JPH::RVec3(worldSpawn.x, worldSpawn.y, worldSpawn.z),
					bodyInterface.GetRotation(partID),
					JPH::EMotionType::Dynamic,
					Layers::MOVING);
				dropSettings.mFriction = 0.5f;
				dropSettings.mLinearVelocity = JPH::Vec3(dir.x, dir.y, dir.z) * kDropSpeed;

				dropIDs.push_back(bodyInterface.CreateAndAddBody(dropSettings, JPH::EActivation::Activate));

				std::cout << "Cubo solto sobre o furo " << (nextHole + kHoleCount - 1) % kHoleCount + 1
					<< " de " << kHoleCount << " (raio do furo " << kHoleRadius
					<< ", cubo de " << kDropHalf * 2.0f << ")" << std::endl;
			}


			// Atualiza a simulação física em passo fixo.
			physicsAccumulator += std::min(deltaTime, 0.1f);

			auto p0 = std::chrono::high_resolution_clock::now();
			int stepsThisFrame = 0;
			while (physicsAccumulator >= physicsStep)
			{
				physicsSystem.Update(
					physicsStep,
					1,
					&tempAllocator,
					&jobSystem
				);
				physicsAccumulator -= physicsStep;
				++stepsThisFrame;
			}
			auto p1 = std::chrono::high_resolution_clock::now();

			const double frameMs = std::chrono::duration<double, std::milli>(p1 - p0).count();
			physicsMsTotal += frameMs;
			physicsMsWorstFrame = std::max(physicsMsWorstFrame, frameMs);
			physicsSteps += stepsThisFrame;
			++physicsFrames;

			if (currentFrame - lastReport >= 1.0f)
			{
				std::cout << "Fisica: media " << physicsMsTotal / physicsFrames << " ms/frame"
					<< " | pior frame " << physicsMsWorstFrame << " ms"
					<< " | " << physicsSteps << " passos";
				if (physicsSteps > 0)
					std::cout << " (" << physicsMsTotal / physicsSteps << " ms/passo)";
				std::cout << " | " << physicsFrames << " frames" << std::endl;

				physicsMsTotal = 0.0;
				physicsMsWorstFrame = 0.0;
				physicsFrames = 0;
				physicsSteps = 0;
				lastReport = currentFrame;
			}


			// input
			processInput(window);

			// o que desenhar neste frame
			const bool drawMesh = !gCollisionView || gOverlay;
			const bool drawCollision = gCollisionView || gOverlay;

			// render (no modo só-colisão o fundo escurece para destacar as linhas)
			if (gCollisionView && !gOverlay)
				glClearColor(0.04f, 0.05f, 0.08f, 1.0f);
			else
				glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
			glm::mat4 view = camera.GetViewMatrix();

			glm::mat4 floorModel = glm::mat4(1.0f);
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

				// cubo (posição E rotação do Jolt)
				ourShader.setMat4("model", BodyModel(bodyInterface, cubeBody->GetID()));
				glDrawArrays(GL_TRIANGLES, 0, 36);

				// parede de caixas
				for (const JPH::BodyID& id : wallIDs)
				{
					ourShader.setMat4("model", BodyModel(bodyInterface, id));
					glDrawArrays(GL_TRIANGLES, 0, 36);
				}

				// NOVO: cubinhos soltos sobre os furos
				for (const JPH::BodyID& id : dropIDs)
				{
					ourShader.setMat4("model", BodyModel(bodyInterface, id) * glm::scale(glm::mat4(1.0f), glm::vec3(kDropHalf * 2.0f)));
					glDrawArrays(GL_TRIANGLES, 0, 36);
				}

				// peça (malha dinâmica)
				glBindVertexArray(partVAO);
				ourShader.setMat4("model", BodyModel(bodyInterface, partID));
				glDrawElements(GL_TRIANGLES, (GLsizei)part.indices.size(), GL_UNSIGNED_INT, 0);

				// NOVO: segunda peça (mesma malha visual)
				if (kSecondPart)
				{
					ourShader.setMat4("model", BodyModel(bodyInterface, part2ID));
					glDrawElements(GL_TRIANGLES, (GLsizei)part.indices.size(), GL_UNSIGNED_INT, 0);
				}
			}


			// =========================================================
			// MODO COLISÃO: o que o Jolt usa de verdade (formas de colisão)
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
				auto shade = [&](JPH::BodyID id, const glm::vec3& c)
					{
						return bodyInterface.IsActive(id) ? c : c * 0.4f;
					};

				const glm::vec3 kBoxColor(0.2f, 1.0f, 0.3f);    // verde: caixa (convexa)
				const glm::vec3 kMeshColor(1.0f, 0.55f, 0.1f);  // laranja: MeshShape
				const glm::vec3 kMassColor(0.2f, 0.85f, 1.0f);  // ciano: caixa só de massa/inércia
				const glm::vec3 kFloorColor(0.6f, 0.65f, 0.75f);
				const glm::vec3 kHullColor(1.0f, 0.9f, 0.2f);   // amarelo: cascos convexos da peça 2

				// pequena folga para as linhas não "brigarem" com as faces texturizadas
				const glm::mat4 kGrow = glm::scale(glm::mat4(1.0f), glm::vec3(1.004f));

				// --- caixas de colisão (arestas) ---
				glBindVertexArray(lineCubeVAO);

				// chão (estático)
				setModel(floorModel * kGrow);
				setColor(kFloorColor);
				glDrawArrays(GL_LINES, 0, 24);

				// cubo
				setModel(BodyModel(bodyInterface, cubeBody->GetID()) * kGrow);
				setColor(shade(cubeBody->GetID(), kBoxColor));
				glDrawArrays(GL_LINES, 0, 24);

				// parede
				for (const JPH::BodyID& id : wallIDs)
				{
					setModel(BodyModel(bodyInterface, id) * kGrow);
					setColor(shade(id, kBoxColor));
					glDrawArrays(GL_LINES, 0, 24);
				}

				// NOVO: cubinhos soltos sobre os furos (também são caixas convexas)
				for (const JPH::BodyID& id : dropIDs)
				{
					setModel(BodyModel(bodyInterface, id) * glm::scale(glm::mat4(1.0f), glm::vec3(kDropHalf * 2.0f)));
					setColor(shade(id, kBoxColor));
					glDrawArrays(GL_LINES, 0, 24);
				}

				// caixa usada SÓ para massa e inércia da peça (não participa da colisão)
				setModel(BodyModel(bodyInterface, partID) * glm::scale(glm::mat4(1.0f), ext));
				setColor(shade(partID, kMassColor));
				glDrawArrays(GL_LINES, 0, 24);

				// NOVO: caixa de massa/inércia da peça 2 (também não colide)
				if (kSecondPart)
				{
					setModel(BodyModel(bodyInterface, part2ID) * glm::scale(glm::mat4(1.0f), ext));
					setColor(shade(part2ID, kMassColor));
					glDrawArrays(GL_LINES, 0, 24);
				}

				// --- malha de triângulos que o Jolt usa na colisão da peça ---
				glBindVertexArray(partVAO);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				glEnable(GL_POLYGON_OFFSET_LINE);
				glPolygonOffset(-1.0f, -1.0f);

				setModel(BodyModel(bodyInterface, partID));
				setColor(shade(partID, kMeshColor));
				glDrawElements(GL_TRIANGLES, (GLsizei)part.indices.size(), GL_UNSIGNED_INT, 0);

				// NOVO: forma de colisão da peça 2
				if (kSecondPart)
				{
					setModel(BodyModel(bodyInterface, part2ID));
					if (kSecondPartMode == 0)
					{
						// mesma MeshShape da peça 1 (laranja)
						setColor(shade(part2ID, kMeshColor));
						glDrawElements(GL_TRIANGLES, (GLsizei)part.indices.size(), GL_UNSIGNED_INT, 0);
					}
					else
					{
						// cascos convexos (amarelo)
						glBindVertexArray(part2VAO);
						setColor(shade(part2ID, kHullColor));
						glDrawArrays(GL_TRIANGLES, 0, part2VertexCount);
					}
				}

				glDisable(GL_POLYGON_OFFSET_LINE);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			}

			// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
			glfwSwapBuffers(window);
			glfwPollEvents();
		}

		// optional: de-allocate all resources once they've outlived their purpose:
		glDeleteVertexArrays(1, &VAO);
		glDeleteBuffers(1, &VBO);
		glDeleteVertexArrays(1, &partVAO);
		glDeleteBuffers(1, &partVBO);
		glDeleteBuffers(1, &partEBO);
		glDeleteVertexArrays(1, &lineCubeVAO);
		glDeleteBuffers(1, &lineCubeVBO);
		glDeleteProgram(flatProgram);
		if (part2VAO) { glDeleteVertexArrays(1, &part2VAO); glDeleteBuffers(1, &part2VBO); }   // NOVO

		// -----------------------------------------
		// Finalização do Jolt Physics
		// -----------------------------------------

		// NOVO: remove a segunda peça
		if (kSecondPart)
		{
			bodyInterface.RemoveBody(part2ID);
			bodyInterface.DestroyBody(part2ID);
		}

		// NOVO: remove os cubinhos do teste do furo
		for (const JPH::BodyID& id : dropIDs)
		{
			bodyInterface.RemoveBody(id);
			bodyInterface.DestroyBody(id);
		}

		// remove a peça e a parede
		bodyInterface.RemoveBody(partID);
		bodyInterface.DestroyBody(partID);

		for (const JPH::BodyID& id : wallIDs)
		{
			bodyInterface.RemoveBody(id);
			bodyInterface.DestroyBody(id);
		}

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


// chamado quando o modo de visualização muda (título da janela + legenda no console)
static void OnViewModeChanged(GLFWwindow* window)
{
	std::string title = "Jolt Physics | ";

	if (gOverlay)
		title += "SOBREPOSICAO: malha + corpo de colisao";
	else if (gCollisionView)
		title += "CORPO DE COLISAO (Jolt)";
	else
		title += "MALHA (visual)";

	title += "  [B = alterna, O = sobrepoe, ESPACO = cubo no furo]";
	glfwSetWindowTitle(window, title.c_str());

	std::cout << "\n=== " << title << " ===" << std::endl;
	if (gCollisionView || gOverlay)
	{
		std::cout << "  Verde   = caixas (BoxShape, forma convexa) - colidem\n"
			<< "  Laranja = malha de triangulos (MeshShape) da peca - colide\n"
			<< "  Ciano   = caixa usada so para massa/inercia da peca - NAO colide\n"
			<< "  Cinza   = chao (estatico)\n"
			<< (kSecondPart && kSecondPartMode != 0
				? "  Amarelo = peca 2: cascos convexos (aproximacao, pois o Jolt nao colide malha x malha)\n" : "")
			<< "  Cores escuras = corpo adormecido (sleeping)" << std::endl;
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

	// B alterna malha <-> colisão; O liga/desliga a sobreposição.
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
