/* Start Header -----------------------------------------------------------------
File: main.cpp
Project: CS550 Engine
Author: Brady Menendez
End Header --------------------------------------------------------------------*/

#include <iostream>
#define GLEW_STATIC
#include "Object.h"
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>
#include "glm\vec3.hpp"
#include "glm\vec4.hpp"
#include "glm\vec2.hpp"
#include <glm\gtc\quaternion.hpp>
#include <glm\mat3x3.hpp>
#include <glm\mat4x4.hpp>
#include <vector>
#include <unordered_map>
#include <random>
#include "Texture.h"
#include "BoundingUtils.h"
#include "FrameRateController.h"
#include "PhysicsManager.h"

bool startRandomLights = true;

// ImGui variables
bool draw_vec_normals = false;
bool draw_face_normals = false;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.0f);

static bool startRotation = false;
static bool toggleDepth = true;
static bool drawLights = false;
static float rotation = 0.15f;
static float TimeAccumulator = 0.0f;

static float col1[3] = { 1.0f,0.0f,0.2f };
static float lColAmb[3] = { 1.0f,0.0f,0.2f };
static float lColDiff[3] = { 1.0f,0.0f,0.2f };
static float lColSpec[3] = { 1.0f,0.0f,0.2f };

static float s_Color[3] = { 1.0f,0.0f,0.2f };
std::uniform_real_distribution<float> unif(0.0f, 1.0f);
std::uniform_int_distribution<int> i_unif(0, 2);
std::mt19937_64 rng;

static float c1 = 0.1f;
static float c2 = 0.08f;
static float c3 = 0.5f;
static float fogC[3] = { 0.4f, 0.4f, 0.4f };
static float ambientC[3] = { 0.1f, 0.1f, 0.1f };
static float innerAngle = 0.0f;
static float outerAngle = 0.0f;
static float falloff = 0.0f;

const char* meshes[] = { "Sphere", "Horse", "Bunny", "Cube" , "Menger Sponge", "Teapot", "Plane"};
const char* texTypes[] = { "Cylindrical", "Spherical", "Planar" };
const char* lTypes[] = { "Point", "Directional", "Spot" };
const char* s_lTypes[] = { "Point", "Directional", "Spot" };
const char* spheres[] = { "Bunny" ,"Horse" , "Teapot", "Menger Sponge", "Sphere", "Cube"};
const char* lightSpheres[] = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16" }; 
const char* renderMode[] = { "Regular", "Position", "Normal", "Diffuse Texture", "Specular Texture" };
static const char* aLights[] = { "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null", "null" };
const char* volumes[] = { "AABB" ,"Bounding Sphere (Centroid)" , "Bounding Sphere (Ritter's)", "Bounding Sphere (Larsson's)", "Bounding Sphere (PCA)", "Bounding Ellipsoid (PCA)", "OBB" };
const char* hierarchies[] = { "TopDown_AABB", "TopDown_BSphere", "BottomUp_AABB", "BottomUp_BSphere" };
const char* levels[] = { "0", "1", "2", "3", "4",  "5", "6", "7"};

static int currentMesh = 2;
static int currentSphere = 0;
static int currentLightSphere = 0;
static int currentLight = 0;
static int activeLight = 0;
static int activeIndex = 0;
static int currTexType = 0;
static int currScenLightType = 0;
static int currRenderMode = 0;
static int currVolume = 0;
static int currHierarchy = 0;
static int currLevel = 0;
static bool currCutoff = 0;

static bool pauseSimulation = 1;
static bool DrawAABB_BVH = 0;

std::unordered_map<std::string, Mesh> meshMap;
std::unordered_map<std::string, Shader> shaderMap;

// constants
const int WIDTH = 1600, HEIGHT = 1200, NUM_ORBIT_LINES = 100;
const float PI = 3.14159f;

// function declarations
void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void UpdateMesh(Object& obj, std::string meshName);
void UpdateShader(Object& obj, std::string shaderName);
static Camera cam(glm::vec3(0, 0, -8), 70.0f, (float)WIDTH / (float)HEIGHT, 0.01f, 1000.0f);
static Transform defaultT;
void SetupOrbit();
void DrawOrbit(Shader& s);
void renderQuad();

unsigned int quadVAO = 0;
unsigned int quadVBO;

// orbit variables
GLuint orbitObject;
GLuint orbitBuffer[1];
unsigned int orbitPointCount;

// light variables
static LightGlobalData lData;
std::unordered_map<int, Light> lights;
std::vector<int> light_indexes;
static int lightCount = 0;

void DrawAABB(Object* obj, Shader& s);
void DrawAABB(AABB bb, Shader& s);
void DrawBSphere(BSphere bs, Shader& s);
void DrawBSphere(Object obj, BSphere bs, Shader& s);
void DrawBSphere_Centroid(Object* obj, Shader& s);
void DrawBVH_AABB(const std::vector<Node>& bvh, Shader& s);

static bool drawVolumes = true;
static bool TreeMode = false;
static bool cutoffMode = false;

std::vector<Node> BottomUpTree_BB;
std::vector<Node> BottomUpTree_BS;
std::vector<std::vector<glm::vec3>> simplices;
static unsigned int currentInd = 0;
void DrawSimplex(Shader& s);

static bool gjkDraw = true;
static bool BVHDraw = true;
static bool velocityDraw = true;

FrameRateController* g_FrameRateController;
PhysicsManager* g_PhysicsManager;

int main()
{
	// Init GLFW
	glfwInit();

	// Set all the required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	// Create a GLFWwindow object that we can use for GLFW's functions
	GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "CS550", nullptr, nullptr);

	int screenWidth, screenHeight;
	glfwGetFramebufferSize(window, &screenWidth, &screenHeight);

	if (nullptr == window)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();

		return EXIT_FAILURE;
	}

  // GLFW Init
  glfwMakeContextCurrent(window);
  glfwSetKeyCallback(window, keyCallback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  //glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);
	
  // GLEW Init
  glewExperimental = GL_TRUE;

	if (GLEW_OK != glewInit())
	{
		std::cout << "Failed to initialize GLEW" << std::endl;
		return EXIT_FAILURE;
	}

	// Define the viewport dimensions
	glViewport(0, 0, screenWidth, screenHeight);

  glEnable(GL_NORMALIZE);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  SetupOrbit();

  g_FrameRateController = new FrameRateController();
  g_FrameRateController->InitializeFrameRateController();
  g_PhysicsManager = new PhysicsManager();

  // textures
  Texture tex0("Textures/metal_roof_diff_512x512.ppm", 0);
  Texture tex1("Textures/metal_roof_spec_512x512.ppm", 1);

  // Load shaders and meshes
  Shader psShader("Shaders/phongShading", "Phong_Shading");
  shaderMap["Phong_Shading"] = psShader; 
  Shader lShader("Shaders/lightShader", "Light");
  shaderMap["Light"] = lShader;
  Shader oShader("Shaders/orbitShader", "Orbit");
  Shader gPassShader("Shaders/geometryPassShader", "G_Pass");
  Shader lPassShader("Shaders/lightingPassShader", "L_Pass");
  Shader dShader("Shaders/debugShader", "Debug");

  Mesh sphereMesh("models/sphere_mid_poly.obj", "Sphere");
  Mesh cMesh("models/cube_low_poly.obj", "Cube");
  Mesh planeMesh("models/plane_low_poly.obj", "Plane", glm::vec3(0.15f, 0.10f, 0.86f));
  meshMap["Sphere"] = sphereMesh;
  meshMap["Cube"] = cMesh;
  meshMap["Plane"] = planeMesh;
  
  lData.c1_ = c1;
  lData.c2_ = c2;
  lData.c3_ = c3;

  unsigned int gBuffer;
  glGenFramebuffers(1, &gBuffer);
  glBindFramebuffer(GL_FRAMEBUFFER, gBuffer);
  unsigned int gPosition, gNormal, gTex, gSpec;
  // position color buffer
  glGenTextures(1, &gPosition);
  glBindTexture(GL_TEXTURE_2D, gPosition);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, WIDTH, HEIGHT, 0, GL_RGB, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gPosition, 0);
  // normal color buffer
  glGenTextures(1, &gNormal);
  glBindTexture(GL_TEXTURE_2D, gNormal);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, WIDTH, HEIGHT, 0, GL_RGB, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, gNormal, 0);

  // color + specular color buffer
  glGenTextures(1, &gTex);
  glBindTexture(GL_TEXTURE_2D, gTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, WIDTH, HEIGHT, 0, GL_RGB, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, gTex, 0);

  glGenTextures(1, &gSpec);
  glBindTexture(GL_TEXTURE_2D, gSpec);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, WIDTH, HEIGHT, 0, GL_RGB, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, gSpec, 0);

  // tell OpenGL which color attachments we'll use (of this framebuffer) for rendering 
  unsigned int attachments[4] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 , GL_COLOR_ATTACHMENT3};
  glDrawBuffers(4, attachments);
  // create and attach depth buffer (renderbuffer)

  unsigned int rboDepth;
  glGenRenderbuffers(1, &rboDepth);
  glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, WIDTH, HEIGHT);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboDepth);
  // finally check if framebuffer is complete
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	  std::cout << "Framebuffer not complete!" << std::endl;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  gPassShader.Bind();
  gPassShader.setInt("tex0", 0);
  gPassShader.setInt("tex1", 1);

  // sets up all objects for the scene
  std::vector<Object*> objects;
  //objects.push_back(Object("Bunny", &meshMap["Bunny"],&gPassShader, &tex0, &tex1, &cam, glm::vec3(-1, 0 , 1), glm::vec3(1, 1, 1), true));
  objects.push_back(new Object("Cube 4", &meshMap["Cube"],&gPassShader, &tex0, &tex1, &cam, glm::vec3(1, 0 ,1), glm::vec3(1, 1, 1), true));
  objects.push_back(new Object("Cube 3", &meshMap["Cube"],&gPassShader, &tex0, &tex1, &cam, glm::vec3(1, 0 ,-1), glm::vec3(1, 1, 1), true));
  objects.push_back(new Object("Cube 2", &meshMap["Cube"],&gPassShader, &tex0, &tex1, &cam, glm::vec3(-1, 0 ,-1), glm::vec3(1.0), true));
  objects.push_back(new Object("Sphere", &meshMap["Sphere"],&gPassShader, &tex0, &tex1, &cam, glm::vec3(0, 0 ,-1), glm::vec3(1.0), true));
  objects.push_back(new Object("Cube", &meshMap["Cube"], &gPassShader, &tex0, &tex1, &cam, glm::vec3(0, 0 ,1), glm::vec3(1.0), true));
  objects.push_back(new Object("Cube 6", &meshMap["Cube"], &gPassShader, &tex0, &tex1, &cam, glm::vec3(-2, 0, -1), glm::vec3(1.0), true));


glm::quat rollRotation = glm::angleAxis(-1.5f, glm::vec3(0, 0, 1));
  objects.back()->GetTransform()->Rotation = rollRotation;

 //objects.push_back(Object("Cube5", &meshMap["Cube"], &gPassShader, &tex0, &tex1, &cam, glm::vec3(-1, 3 ,-1), glm::vec3(1.0)));
  objects.push_back(new Object("Floor", &meshMap["Cube"], &gPassShader, &tex0, &tex1, &cam, glm::vec3(0, -4 ,0), glm::vec3(15, 1, 15), true));

  objects.back()->SetPlatform();
  objects.back()->SetTexMode(2);
  objects.back()->GetRigidBody()->mass_ = 10.0f;

  // creating volume hierarchies
  BottomUpTree_BB = BottomUpBVTree(objects, objects.size());
  BottomUpTree_BS = BottomUpBVTree_BSphere(objects, objects.size());
  
  lPassShader.Bind();
  lPassShader.setInt("gPosition", 0);
  lPassShader.setInt("gNormal", 1);
  lPassShader.setInt("gTex", 2);
  lPassShader.setInt("gSpec", 3);

  std::vector<Object> lightObjects;

  float radius = 2.0f;
  for (int i = 0; i < 16; i++)
  {
	  lightObjects.push_back(Object(std::to_string(i + 1), &meshMap["Sphere"], &shaderMap["Light"], &tex0, &tex1, &cam, glm::vec3(radius * cosf(2 * PI * i / 16), 0, radius * sinf(2 * PI * i / 16)), glm::vec3(0.1f, 0.1f, 0.1f)));
	  lightObjects[i].SetLightColor(glm::vec3(0.0f));
  }


  if (startRandomLights)
  {
	  // start as randomized lights
	  lights.clear();
	  light_indexes.clear();
	  lightCount = 0;

	  for (int i = 0; i < 16; ++i)
	  {

		  lightObjects[i].SetShader(&shaderMap["Light"]);
		  glm::vec3 randColor = glm::vec3(unif(rng), unif(rng), unif(rng));
		  int randType = i_unif(rng);
		  lightObjects[i].SetLightColor(randColor);

		  if (randType == Light::POINT)
			  lights[i] = Light::MakePointLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor);
		  else if (randType == Light::DIRECTIONAL)
			  lights[i] = Light::MakeDirectionalLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor);
		  else if (randType == Light::SPOTLIGHT)
			  lights[i] = Light::MakeSpotLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor, 20.0f, 30.0f, 1.0f);

		  lights[i].isEnabled_ = true;
		  lightCount++;
		  light_indexes.push_back(std::stoi(lightObjects[i].GetName()) - 1);
	  }
  }

  GLuint vao;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  // Cube 1x1x1, centered on origin
  GLfloat vertices[] = {
	-0.5, -0.5, -0.5, 1.0,
	 0.5, -0.5, -0.5, 1.0,
	 0.5,  0.5, -0.5, 1.0,
	-0.5,  0.5, -0.5, 1.0,
	-0.5, -0.5,  0.5, 1.0,
	 0.5, -0.5,  0.5, 1.0,
	 0.5,  0.5,  0.5, 1.0,
	-0.5,  0.5,  0.5, 1.0,
  };
  GLuint vbo_vertices;
  glGenBuffers(1, &vbo_vertices);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  GLushort elements[] = {
	0, 1, 2, 3,
	4, 5, 6, 7,
	0, 4, 1, 5, 2, 6, 3, 7
  };
  GLuint ibo_elements;
  glGenBuffers(1, &ibo_elements);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(
	  0,
	  4,
	  GL_FLOAT,
	  GL_FALSE,
	  0,
	  0
  );


  // ImGui Init
  ImGui_ImplGlfwGL3_Init(window, true);

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
      // Checks for key events
      glfwPollEvents();

	  // Clear Buffer
	  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	  g_FrameRateController->UpdateFrameTime();

    // ImGui Code
    {
      ImGui_ImplGlfwGL3_NewFrame();
	  ImGui::TextColored(ImVec4(1, 1, 0, 1), "CAMERA CONTROLS :");
      ImGui::Text("WASD to MOVE, ARROW KEYS to ROTATE, SPACE TO RESET");
      ImGui::Separator();

	  // bounding volume UI section
	  ImGui::TextColored(ImVec4(1, 1, 0, 1), "DEBUG DRAWING OPTIONS :");
	  ImGui::Checkbox("DRAW COLLIDERS", &drawVolumes);
	  
	
	  ImGui::Checkbox("DRAW BROAD PHASE", &BVHDraw);

	  ImGui::Checkbox("DRAW SIMPLICES", &gjkDraw);
	  ImGui::Checkbox("DRAW LINEAR VELOCITY", &velocityDraw);


	  ImGui::Separator();

	  ImGui::TextColored(ImVec4(1, 1, 0, 1), "SIMULATION CONTROLS :");


	  if (ImGui::Button("Play"))
		  pauseSimulation = false;

	  ImGui::SameLine();

	  if (ImGui::Button("Pause"))
		  pauseSimulation = true;

	  if (ImGui::Button("Reset"))
	  {
		  for (auto obj : objects)
			  obj->GetTransform()->SetPosition(obj->initPos_);
	  }


	  ImGui::Separator();

      static float f = 0.0f;
      ImGui::TextColored(ImVec4(1, 1, 0, 1), "DRAWING OPTIONS :");
      ImGui::Checkbox("VERTEX NORMALS", &draw_vec_normals);
      ImGui::Checkbox("FACE NORMALS", &draw_face_normals);
      ImGui::Separator();

      ImGui::TextColored(ImVec4(1, 1, 0, 1), "SELECTED OBJECT :");
      ImGui::Combo("SELECTED", &currentSphere, spheres, 6);

	  // texture variables
	  currTexType = objects[currentSphere]->GetTexMode();
	  ImGui::Combo("Texture Wrap Type", &currTexType, texTypes, 3);
	  objects[currentSphere]->SetTexMode(currTexType);

      ImGui::Separator();

      ImGui::TextColored(ImVec4(1, 1, 0, 1), "GLOBAL LIGHT VALUES :");

      // attenuation coefficients
      ImGui::Text("Distance Attenuation Coefficients : ");
      c1 = lData.c1_;
      ImGui::InputFloat("C1", &c1, 0.01f, 0.1f); 
      lData.c1_ = c1;

      c2 = lData.c2_;
      ImGui::InputFloat("C2", &c2, 0.01f, 0.1f); 
      lData.c2_ = c2;

      c3 = lData.c3_;
      ImGui::InputFloat("C3", &c3, 0.01f, 0.1f);
      lData.c3_ = c3;

      // global fog
      fogC[0] = lData.fog_[0];
      fogC[1] = lData.fog_[1];
      fogC[2] = lData.fog_[2];
      ImGui::ColorEdit3("Fog Color", fogC);
      lData.fog_[0] = fogC[0];
      lData.fog_[1] = fogC[1];
      lData.fog_[2] = fogC[2];

      // global ambient
      ambientC[0] = lData.ambient_[0];
      ambientC[1] = lData.ambient_[1];
      ambientC[2] = lData.ambient_[2];
      ImGui::ColorEdit3("Ambient Color", ambientC);
      lData.ambient_[0] = ambientC[0];
      lData.ambient_[1] = ambientC[1];
      lData.ambient_[2] = ambientC[2];

      ImGui::Separator();

	  ImGui::TextColored(ImVec4(1, 1, 0, 1), "LIGHT SCENARIO SELECT :");
	  ImGui::ColorEdit3("Lights Color", s_Color);
	  ImGui::Combo("Lights Type", &currScenLightType, s_lTypes, 3);

	  // scenario change buttons
	  if (ImGui::Button("Switch to Scenario 1 : All Lights -> Same Color, Same Type"))
	  {
		  lights.clear();
		  light_indexes.clear();
		  lightCount = 0;

		  // sets new lights
		  for (int i = 0; i < 16; ++i)
		  {
			  lightObjects[i].SetShader(&shaderMap["Light"]);
			  lightObjects[i].SetLightColor(glm::vec3(s_Color[0], s_Color[1], s_Color[2]));

			  if (currScenLightType == Light::POINT)
				  lights[i] = Light::MakePointLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), glm::vec3(s_Color[0], s_Color[1], s_Color[2]));
			  else if (currScenLightType == Light::DIRECTIONAL)
				  lights[i] = Light::MakeDirectionalLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), glm::vec3(s_Color[0], s_Color[1], s_Color[2]));
			  else if (currScenLightType == Light::SPOTLIGHT)
				  lights[i] = Light::MakeSpotLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), glm::vec3(s_Color[0], s_Color[1], s_Color[2]), 5.0f, 10.0f, 1.0f);

			  lights[i].isEnabled_ = true;
			  lightCount++;
			  light_indexes.push_back(std::stoi(lightObjects[i].GetName()) - 1);
		  }
	  }

	  if (ImGui::Button("Switch to Scenario 2 : All Lights -> Same Type, Random Colors"))
	  {
		  lights.clear();
		  light_indexes.clear();
		  lightCount = 0;

		  // sets new lights
		  for (int i = 0; i < 16; ++i)
		  {
			  lightObjects[i].SetShader(&shaderMap["Light"]);
			  glm::vec3 randColor = glm::vec3(unif(rng), unif(rng), unif(rng));
			  lightObjects[i].SetLightColor(randColor);

			  if (currScenLightType == Light::POINT)
				  lights[i] = Light::MakePointLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor);
			  else if (currScenLightType == Light::DIRECTIONAL)
				  lights[i] = Light::MakeDirectionalLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor);
			  else if (currScenLightType == Light::SPOTLIGHT)
				  lights[i] = Light::MakeSpotLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor, 20.0f, 30.0f, 1.0f);

			  lights[i].isEnabled_ = true;
			  lightCount++;
			  light_indexes.push_back(std::stoi(lightObjects[i].GetName()) - 1);
		  }
	  }

	  if (ImGui::Button("Switch to Scenario 3 : RANDOMIZE EVERYTHING"))
	  {
		  lights.clear();
		  light_indexes.clear();
		  lightCount = 0;

		  for (int i = 0; i < 16; ++i)
		  {

			  lightObjects[i].SetShader(&shaderMap["Light"]);
			  glm::vec3 randColor = glm::vec3(unif(rng), unif(rng), unif(rng));
			  int randType = i_unif(rng);
			  lightObjects[i].SetLightColor(randColor);

			  if (randType == Light::POINT)
				  lights[i] = Light::MakePointLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor);
			  else if (randType == Light::DIRECTIONAL)
				  lights[i] = Light::MakeDirectionalLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor);
			  else if (randType == Light::SPOTLIGHT)
				  lights[i] = Light::MakeSpotLight(lightObjects[i].GetName(), lightObjects[i].GetTransform()->GetPosition(), randColor, 20.0f, 30.0f, 1.0f);

			  lights[i].isEnabled_ = true;
			  lightCount++;
			  light_indexes.push_back(std::stoi(lightObjects[i].GetName()) - 1);
		  }
	  }

	  ImGui::Separator();

      ImGui::TextColored(ImVec4(1, 1, 0, 1), "LIGHT CONTROL :");
      
	  ImGui::Combo("Select Light To Enable", &currentLightSphere, lightSpheres, 16);

	  ImGui::Separator();

	  // new light commands
      if(!lights[currentLightSphere].isEnabled_)
      { 

        ImGui::Combo("Type", &currentLight, lTypes, 3);

        ImGui::ColorEdit3("Color", col1);

        if (ImGui::Button("Add as new light"))
        {
			lightObjects[currentLightSphere].SetShader(&shaderMap["Light"]);
			lightObjects[currentLightSphere].SetLightColor(glm::vec3(col1[0], col1[1], col1[2]));
        
          if (currentLight == Light::POINT)
           lights[currentLightSphere] = Light::MakePointLight(lightObjects[currentLightSphere].GetName(), lightObjects[currentLightSphere].GetTransform()->GetPosition(), glm::vec3(col1[0], col1[1], col1[2]));
          else if (currentLight == Light::DIRECTIONAL)
            lights[currentLightSphere] = Light::MakeDirectionalLight(lightObjects[currentLightSphere].GetName(), lightObjects[currentLightSphere].GetTransform()->GetPosition(), glm::vec3(col1[0], col1[1], col1[2]));
          else if (currentLight == Light::SPOTLIGHT)
            lights[currentLightSphere] = Light::MakeSpotLight(lightObjects[currentLightSphere].GetName(), lightObjects[currentLightSphere].GetTransform()->GetPosition(), glm::vec3(col1[0], col1[1], col1[2]), 20.0f, 30.0f, 1.0f);
          
		  lights[currentLightSphere].isEnabled_ = true;
          lightCount++;

		  int lightInd = std::stoi(lightObjects[currentLightSphere].GetName()) - 1;

          light_indexes.push_back(lightInd);
          activeIndex = lightCount - 1;
        }
      }

	  // light count
      if (lightCount > 0)
      { 
        std::string num = std::to_string(lightCount);

        if (lightCount == 1)
          num += " active light";
        else
          num += " active lights";
        
        ImGui::TextColored(ImVec4(1, 0.65f, 0, 1), "Light Editor :  "); ImGui::SameLine(); ImGui::Text(num.c_str());

		ImGui::Text("Current: "); ImGui::SameLine(); ImGui::Text(std::to_string(light_indexes[activeIndex] + 1).c_str());

		if (lightCount > 1)
		{
			if (ImGui::Button("Next Light"))
			{
				if (activeIndex == lightCount - 1) activeIndex = 0;
				else                               activeIndex++;
			}
		}

      }
      else
      {
        ImGui::Text("No active lights");
      }

	  // current light display
      if (lightCount > 0)
      {
        if (lights[light_indexes[activeIndex]].type_ == Light::POINT)
          ImGui::Text("Type: POINT");
        else if (lights[light_indexes[activeIndex]].type_ == Light::DIRECTIONAL)
          ImGui::Text("Type: DIRECTIONAL");
        else if (lights[light_indexes[activeIndex]].type_ == Light::SPOTLIGHT)
          ImGui::Text("Type: SPOTLIGHT");

        lColAmb[0] = lights[light_indexes[activeIndex]].ambientColor_.x;
        lColAmb[1] = lights[light_indexes[activeIndex]].ambientColor_.y;
        lColAmb[2] = lights[light_indexes[activeIndex]].ambientColor_.z;
        ImGui::ColorEdit3("Current Ambient Color", lColAmb);
        lights[light_indexes[activeIndex]].ambientColor_.x = lColAmb[0];
        lights[light_indexes[activeIndex]].ambientColor_.y = lColAmb[1];
        lights[light_indexes[activeIndex]].ambientColor_.z = lColAmb[2];

        lColDiff[0] = lights[light_indexes[activeIndex]].diffuseColor_.x;
        lColDiff[1] = lights[light_indexes[activeIndex]].diffuseColor_.y;
        lColDiff[2] = lights[light_indexes[activeIndex]].diffuseColor_.z;
        ImGui::ColorEdit3("Current Diffuse Color", lColDiff);
        lights[light_indexes[activeIndex]].diffuseColor_.x = lColDiff[0];
        lights[light_indexes[activeIndex]].diffuseColor_.y = lColDiff[1];
        lights[light_indexes[activeIndex]].diffuseColor_.z = lColDiff[2];

        lightObjects[light_indexes[activeIndex]].SetLightColor(glm::vec3(lColDiff[0], lColDiff[1], lColDiff[2]));

        lColSpec[0] = lights[light_indexes[activeIndex]].specularColor_.x;
        lColSpec[1] = lights[light_indexes[activeIndex]].specularColor_.y;
        lColSpec[2] = lights[light_indexes[activeIndex]].specularColor_.z;
        ImGui::ColorEdit3("Current Specular Color", lColSpec);
        lights[light_indexes[activeIndex]].specularColor_.x = lColSpec[0];
        lights[light_indexes[activeIndex]].specularColor_.y = lColSpec[1];
        lights[light_indexes[activeIndex]].specularColor_.z = lColSpec[2];

        if (lights[light_indexes[activeIndex]].type_ == Light::SPOTLIGHT)
        {
          innerAngle = glm::degrees(lights[light_indexes[activeIndex]].innerAngle_);
          ImGui::InputFloat("Inner Angle", &innerAngle, 5.0f);
          lights[light_indexes[activeIndex]].innerAngle_ = glm::radians(innerAngle);

          outerAngle = glm::degrees(lights[light_indexes[activeIndex]].outerAngle_);
          ImGui::InputFloat("Outer Angle", &outerAngle, 5.0f);
          lights[light_indexes[activeIndex]].outerAngle_ = glm::radians(outerAngle);

          falloff = lights[light_indexes[activeIndex]].spotFalloff_;
          ImGui::InputFloat("Spot Falloff", &falloff, 0.05f);
          lights[light_indexes[activeIndex]].spotFalloff_ = falloff;
        }

        if (ImGui::Button("Remove Light"))
        {
		  lightObjects[light_indexes[activeIndex]].SetLightColor(glm::vec3(0.0f));
		  lights[light_indexes[activeIndex]].isEnabled_ = false;
		  int remIndex = light_indexes[activeIndex];
		  lights.erase(remIndex);
		  light_indexes.erase(std::find(light_indexes.begin(), light_indexes.end(), remIndex));
          lightCount--;
		  activeIndex = 0;
        }
      }

	  //ImGui::SameLine();
	  if (ImGui::Button("Kill All Lights"))
	  {
		  for (int i = 0; i < 16; ++i)
		  {
			  lightObjects[i].SetLightColor(glm::vec3(0.0f));
			  lights[i].isEnabled_ = false;
		  }

		  lights.clear();
		  lightCount = 0;
	  }

	  ImGui::Separator();
	  ImGui::Checkbox("DRAW LIGHTS", &drawLights);
      ImGui::Checkbox("ROTATE LIGHTS", &startRotation);
      ImGui::Separator();
	  ImGui::TextColored(ImVec4(1, 1, 0, 1), "DEFERRED SHADING SETTINGS:");
	  ImGui::Checkbox("TOGGLE DEPTH COPYING", &toggleDepth);
	  ImGui::Combo("RENDER TARGET", &currRenderMode, renderMode, 5);
	  ImGui::Separator();

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    }




	if (!pauseSimulation)
	{
		TimeAccumulator += g_FrameRateController->DeltaTime;

		while (TimeAccumulator >= g_FrameRateController->FixedDelta)
		{
			TimeAccumulator -= g_FrameRateController->FixedDelta;

			g_PhysicsManager->Update();
		}


	}
		

	// GEOMETRY PASS
	glBindFramebuffer(GL_FRAMEBUFFER, gBuffer);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	gPassShader.Bind();

	for (Object* obj : objects)
	{
		obj->DrawDeferred();
	}	

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// LIGHTING PASS
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	lPassShader.Bind();
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, gPosition);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, gNormal);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, gTex);
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, gSpec);
	Transform e;
	lPassShader.Update(&e, cam, lights, lData);
	lPassShader.setInt("renderTargetMode", currRenderMode);
	renderQuad();
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_DEPTH_BUFFER_BIT);

	// DEPTH BUFFER COPY (IF ENABLED)
	if (toggleDepth)
	{
		glBindFramebuffer(GL_READ_FRAMEBUFFER, gBuffer);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		glBlitFramebuffer(0, 0, WIDTH, HEIGHT, 0, 0, WIDTH, HEIGHT, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	BottomUpTree_BB = BottomUpBVTree(objects, objects.size());

	if (BVHDraw)
		DrawBVH_AABB(BottomUpTree_BB, dShader);


	if (gjkDraw)
		DrawSimplex(dShader);


	for (Object* obj : objects)
	{
		// normal drawing
		obj->DrawDebug(draw_vec_normals, draw_face_normals, velocityDraw);

		// volume drawing (single objects)
		if (drawVolumes && !TreeMode)
		{
			switch (currVolume)
			{
			  case 0:
			  DrawAABB(obj, dShader);
			  break;

			  case 1:
			  DrawBSphere_Centroid(obj,dShader);
			  break;

			}
		}
	}


    // Draw lights

	if (drawLights)
	{
		for (Object obj : lightObjects)
		{
			std::string name = obj.GetName();
			glm::vec3 rotPos = glm::rotate(rotation, glm::vec3(0, 1, 0)) * glm::vec4(obj.GetTransform()->GetPosition(), 1.0f);

			if (startRotation)
				obj.GetTransform()->SetPosition(rotPos);

			if (lights.find(std::stoi(name) - 1) != lights.end())
				lights[std::stoi(obj.GetName()) - 1].position_ = rotPos;

			obj.Draw(lights, lData, false, false);
			rotation += 0.00006f;
		}
	}

      ImGui::Render();
	  glfwSwapBuffers(window);
	}

  // shutdown calls
  ImGui_ImplGlfwGL3_Shutdown();
  glfwTerminate();

  return EXIT_SUCCESS;
}

// glfw key callback function
void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  // key and camera function mapping
  if (key == GLFW_KEY_W && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "W Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::FORWARD, 0.01f);
  }

  if (key == GLFW_KEY_A && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "A Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::LEFT, 0.01f);
  }

  if (key == GLFW_KEY_S && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "S Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::BACKWARD, 0.01f);
  }

  if (key == GLFW_KEY_D && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "D Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::RIGHT, 0.01f);
  }

  if (key == GLFW_KEY_UP && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "Up Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::UP_A, 0.01f);
  }

  if (key == GLFW_KEY_DOWN && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "Down Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::DOWN_A, 0.01f);
  }

  if (key == GLFW_KEY_LEFT && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "Left Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::LEFT_A, 0.01f);
  }

  if (key == GLFW_KEY_RIGHT && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
    //std::cout << "Right Key Pressed" << std::endl;
    cam.ProcessKeyboard(Camera_Movement::RIGHT_A, 0.01f);
  }

  if (key == GLFW_KEY_SPACE && (action == GLFW_REPEAT || action == GLFW_PRESS))
  {
	  //std::cout << "Space Key Pressed" << std::endl;
	  cam.ProcessKeyboard(Camera_Movement::SPACE, 0.01f);
  }

}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);

		std::cout << "Mouse Pos: X: " << xpos << ", Y: " << ypos << std::endl;
	}


	std::cout << "pain" << std::endl;
}

void DrawSimplex(Shader& s)
{
	// draws a simplex
	std::vector<glm::vec3> simplex;

	simplex.push_back(g_PhysicsManager->currSimplex.a.MHVertex_);
	simplex.push_back(g_PhysicsManager->currSimplex.b.MHVertex_);
	simplex.push_back(g_PhysicsManager->currSimplex.c.MHVertex_);
	simplex.push_back(g_PhysicsManager->currSimplex.d.MHVertex_);

	if (simplex.empty())
		return;


	int pointCount = simplex.size();

	std::vector<glm::vec3> points2;

	for (auto& p1 : simplex)
	{
		for (auto& p2 : simplex)
		{
			points2.push_back(p1);
			points2.push_back(p2);
		}
	}


	GLuint simplexObject;
	GLuint simplexBuffer[1];

	// orbit lines VAO
	glGenVertexArrays(1, &simplexObject);
	glBindVertexArray(simplexObject);

	glGenBuffers(1, simplexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, simplexBuffer[0]);
	glBufferData(GL_ARRAY_BUFFER, points2.size() * sizeof(points2[0]), &points2[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);

	s.Bind();
	s.Update(&defaultT, cam, lights, lData);

	glLineWidth(100);
	glEnable(GL_LINE_SMOOTH);


	glBindVertexArray(simplexObject);
	glDrawArrays(GL_LINE_LOOP, 0, points2.size());
	glBindVertexArray(0);

	glDisable(GL_LINE_SMOOTH);
	glLineWidth(1);
}

void SetupOrbit()
{
  float radius = 2.0f;

  std::vector<glm::vec3> orbitPoints;

  for (int i = 0; i < NUM_ORBIT_LINES; i++)
    orbitPoints.push_back(glm::vec3(radius * cosf(2 * PI * i / NUM_ORBIT_LINES), 0, radius * sinf(2 * PI * i / NUM_ORBIT_LINES)));

  orbitPointCount = orbitPoints.size();

  // orbit lines VAO
  glGenVertexArrays(1, &orbitObject);
  glBindVertexArray(orbitObject);

  glGenBuffers(1, orbitBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, orbitBuffer[0]);
  glBufferData(GL_ARRAY_BUFFER, orbitPoints.size() * sizeof(orbitPoints[0]), &orbitPoints[0], GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glBindVertexArray(0);
}

void DrawOrbit(Shader& s)
{
  s.Bind();
  s.Update(&defaultT, cam, lights, lData);

  glLineWidth(100);
  glEnable(GL_LINE_SMOOTH);


  glBindVertexArray(orbitObject);
  glDrawArrays(GL_LINE_LOOP, 0, orbitPointCount);
  glBindVertexArray(0);

  glDisable(GL_LINE_SMOOTH);
  glLineWidth(1);
}

void UpdateMesh(Object& obj, std::string meshName)
{
  if (meshMap.find(meshName) == meshMap.end()) return;

  if (obj.GetMeshName() != meshName)
    obj.SetMesh(&meshMap[meshName]);
}

void UpdateShader(Object& obj, std::string shaderName)
{
  if (shaderMap.find(shaderName) == shaderMap.end()) return;

  if (obj.GetShaderName() != shaderName)
    obj.SetShader(&shaderMap[shaderName]);
}

void renderQuad()
{
	if (quadVAO == 0)
	{
		float quadVertices[] = {
			// positions        // texture Coords
			-1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
			-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			 1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
			 1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		};
		// setup plane VAO
		glGenVertexArrays(1, &quadVAO);
		glGenBuffers(1, &quadVBO);
		glBindVertexArray(quadVAO);
		glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	}
	glBindVertexArray(quadVAO);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}

void DrawAABB(Object* obj, Shader& s)
{
	s.Bind();

	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// Cube 1x1x1, centered on origin
	GLfloat vertices[] = {
	  -0.5, -0.5, -0.5, 1.0,
	   0.5, -0.5, -0.5, 1.0,
	   0.5,  0.5, -0.5, 1.0,
	  -0.5,  0.5, -0.5, 1.0,
	  -0.5, -0.5,  0.5, 1.0,
	   0.5, -0.5,  0.5, 1.0,
	   0.5,  0.5,  0.5, 1.0,
	  -0.5,  0.5,  0.5, 1.0,
	};
	GLuint vbo_vertices;
	glGenBuffers(1, &vbo_vertices);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLushort elements[] = {
	  0, 1, 2, 3,
	  4, 5, 6, 7,
	  0, 4, 1, 5, 2, 6, 3, 7
	};
	GLuint ibo_elements;
	glGenBuffers(1, &ibo_elements);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(
		0,  
		4,                 
		GL_FLOAT,           
		GL_FALSE,           
		0,                  
		0                   
	);

	// getting transform data, to draw the AABB
	AABB bb = *obj->GetAABB();
	glm::vec4 center = obj->GetTransform()->GetModel() * glm::vec4(bb.center_, 1.0f);
	glm::mat4 transform = glm::translate(glm::mat4(1), glm::vec3(center)) * glm::mat4_cast(obj->GetTransform()->Rotation) * glm::scale(glm::mat4(1), bb.size_);
	glm::mat4 m = cam.GetViewProjection() * transform;

	s.setTransform(m);

	if (bb.isColliding_)
		s.setInt("colliding", 0);
	else
		s.setInt("colliding", 1);


	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, 0);
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, (GLvoid*)(4 * sizeof(GLushort)));
	glDrawElements(GL_LINES, 8, GL_UNSIGNED_SHORT, (GLvoid*)(8 * sizeof(GLushort)));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &vbo_vertices);
	glDeleteBuffers(1, &ibo_elements);
	glDeleteVertexArrays(1, &vao);
}

void DrawAABB(AABB bb, Shader& s)
{
	//std::cout << bb.isColliding_ << std::endl;


	s.Bind();
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// Cube 1x1x1, centered on origin
	GLfloat vertices[] = {
	  -0.5, -0.5, -0.5, 1.0,
	   0.5, -0.5, -0.5, 1.0,
	   0.5,  0.5, -0.5, 1.0,
	  -0.5,  0.5, -0.5, 1.0,
	  -0.5, -0.5,  0.5, 1.0,
	   0.5, -0.5,  0.5, 1.0,
	   0.5,  0.5,  0.5, 1.0,
	  -0.5,  0.5,  0.5, 1.0,
	};
	GLuint vbo_vertices;
	glGenBuffers(1, &vbo_vertices);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLushort elements[] = {
	  0, 1, 2, 3,
	  4, 5, 6, 7,
	  0, 4, 1, 5, 2, 6, 3, 7
	};
	GLuint ibo_elements;
	glGenBuffers(1, &ibo_elements);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(
		0,  
		4,                  
		GL_FLOAT,          
		GL_FALSE,           
		0,                 
		0                   
	);


	glm::mat4 transform = glm::translate(glm::mat4(1), glm::vec3(bb.center_)) * glm::scale(glm::mat4(1), bb.size_);
	glm::mat4 m = cam.GetViewProjection() * transform;
	s.setTransform(m);

	if (bb.isColliding_)
		s.setInt("colliding", 1);
	else
		s.setInt("colliding", 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, 0);
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, (GLvoid*)(4 * sizeof(GLushort)));
	glDrawElements(GL_LINES, 8, GL_UNSIGNED_SHORT, (GLvoid*)(8 * sizeof(GLushort)));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteBuffers(1, &vbo_vertices);
	glDeleteBuffers(1, &ibo_elements);
	glDeleteVertexArrays(1, &vao);
}

void DrawBSphere_Centroid(Object* obj, Shader& s)
{
	s.Bind();
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	std::vector<glm::vec3> verts = obj->GetBSphere().positions_;
	std::vector<unsigned int> inds = obj->GetBSphere().indices_;
	GLuint vbo_vertices;
	glGenBuffers(1, &vbo_vertices);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(verts[0]), &verts[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLuint ibo_elements;
	glGenBuffers(1, &ibo_elements);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, inds.size() * sizeof(inds[0]), &inds[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(
		0,  
		3,                  
		GL_FLOAT,           
		GL_FALSE,           
		0,                  
		0                  
	);


	AABB bb = *obj->GetAABB();

	glm::vec4 center = obj->GetTransform()->GetModel() * glm::vec4(bb.center_, 1.0f);


	glm::mat4 transform = glm::translate(glm::mat4(1), glm::vec3(center));
	glm::mat4 m = cam.GetViewProjection() * transform;

	s.setTransform(m);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);

	glDrawElements(GL_LINE_LOOP, inds.size() * 4, GL_UNSIGNED_INT, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteBuffers(1, &vbo_vertices);
	glDeleteBuffers(1, &ibo_elements);
}

void DrawBSphere(BSphere bs, Shader& s)
{
	s.Bind();

	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	std::vector<glm::vec3> verts = bs.positions_;
	std::vector<unsigned int> inds = bs.indices_;
	GLuint vbo_vertices;
	glGenBuffers(1, &vbo_vertices);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(verts[0]), &verts[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLuint ibo_elements;
	glGenBuffers(1, &ibo_elements);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, inds.size() * sizeof(inds[0]), &inds[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(
		0,  
		3,                 
		GL_FLOAT,           
		GL_FALSE,           
		0,                 
		0                  
	);

	glm::mat4 transform = glm::translate(glm::mat4(1), bs.center_);
	glm::mat4 m = cam.GetViewProjection() * transform;

	s.setTransform(m);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);

	glDrawElements(GL_LINE_LOOP, inds.size() * 4, GL_UNSIGNED_INT, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteBuffers(1, &vbo_vertices);
	glDeleteBuffers(1, &ibo_elements);
}



void DrawBSphere(Object obj, BSphere bs, Shader& s)
{
	s.Bind();
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	std::vector<glm::vec3> verts = bs.positions_;
	std::vector<unsigned int> inds = bs.indices_;
	GLuint vbo_vertices;
	glGenBuffers(1, &vbo_vertices);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(verts[0]), &verts[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLuint ibo_elements;
	glGenBuffers(1, &ibo_elements);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, inds.size() * sizeof(inds[0]), &inds[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(
		0,  
		3,                  
		GL_FLOAT,         
		GL_FALSE,           
		0,                 
		0                   
	);

	glm::vec4 center = obj.GetTransform()->GetModel() * glm::vec4(bs.center_, 1.0f);
	glm::mat4 transform = glm::translate(glm::mat4(1), glm::vec3(center)) * glm::scale(glm::vec3(1.0f));
	glm::mat4 m = cam.GetViewProjection() * transform;

	s.setTransform(m);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);

	glDrawElements(GL_LINE_LOOP, inds.size() * 4, GL_UNSIGNED_INT, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteBuffers(1, &vbo_vertices);
	glDeleteBuffers(1, &ibo_elements);
}

// draws the corresponding level of the bvh
void DrawBVH_AABB(const std::vector<Node>& bvh, Shader& s)
{
	for (const Node& node : bvh)
		DrawAABB(node.bb_, s);
}


// draws the corresponding level of the bvh
void DrawBVHLevel_AABB(const std::vector<Node>& bvh, int level, Shader& s)
{
	for (const Node& node : bvh)
	{
		if (node.level_ == level)
			DrawAABB(node.bb_, s);
	}
}

// draws the corresponding level of the bvh
void DrawBVHLevel_BS(const std::vector<Node>& bvh, int level, Shader& s)
{
	for (const Node& node : bvh)
	{
		if (node.level_ == level)
			DrawBSphere(node.bs_, s);
	}
}

