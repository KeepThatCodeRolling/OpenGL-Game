#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <glad.c>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Shader.h"
#include "camera.h"
#include "Model.h"
#include <iostream>

#include "btBulletDynamicsCommon.h"

//Class GameObject - This was made so that we can lessen the amount of code of every model movement
//---------------------------------------------
class GameObject {
private:
     Model modelLocation;
     glm::vec3 position;
     glm::vec3 scale;
     glm::vec3 rotation;
     glm::mat4 modelMatrix;

     void updateModelMatrix() {
          modelMatrix = glm::mat4(1.0f);
          modelMatrix = glm::translate(modelMatrix, position);
          modelMatrix = glm::rotate(modelMatrix, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
          modelMatrix = glm::rotate(modelMatrix, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
          modelMatrix = glm::rotate(modelMatrix, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
          modelMatrix = glm::scale(modelMatrix, scale);
     }
public:
     GameObject(Model& modelLoc,glm::vec3 pos = glm::vec3(0.0f), glm::vec3 scl = glm::vec3(1.0f), glm::vec3 rot = glm::vec3(0.0f))
          :modelLocation(modelLoc), position(pos), scale(scl), rotation(rot) {
          updateModelMatrix();
     }
     void setPosition(const glm::vec3& pos) {
          position = pos;
          updateModelMatrix();
     }
     void setScale(const glm::vec3& scl) {
          scale = scl;
          updateModelMatrix();
     }
     void setRotation(const glm::vec3 rot) {
          rotation = rot;
          updateModelMatrix();
     }

     glm::vec3 getPosition() const { return position; }
     glm::vec3 getScale() const { return scale; }
     glm::vec3 getRotation() const { return rotation; }

     void render(Shader& shader) {
          shader.setMat4("model", modelMatrix);
          modelLocation.Draw(shader);
     }
};
//---------------------------------------------


class BuildPhysics {
private:

public:
     BuildPhysics() {
          btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
          btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
          btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
          btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
          btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
     }
};


const int SCR_Height = 1080;
const int SCR_Width = SCR_Height * (16.0f/9.0f);

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_callback(GLFWwindow* window, double xoffset, double yoffset);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

//camera
Camera camera(glm::vec3(0.0f, 2.0f, 15.0f));
float lastX = SCR_Width / 2.0f;
float lastY = SCR_Height / 2.0f;
bool firstMouse = true;

//timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main()
{
     glfwInit();
     glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
     glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
     glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

     GLFWwindow* window = glfwCreateWindow(SCR_Width, SCR_Height, "Da Window", NULL, NULL);
     if (!window)
     {
          std::cout << "Failed to make GLFW Window" << std::endl;
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
          std::cout << "Failed to initialize GLAD" << std::endl;
          return -1;
     }
     stbi_set_flip_vertically_on_load(true);
     glEnable(GL_DEPTH_TEST);

     double groundOriginVal = 0;
     double objOriginVal = 10;

     //Initiaize physics
     //---------------------------------------------
     int i;
     //need configuration, dispatcher, broadphase, solver
     btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();

     btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);

     btBroadphaseInterface* overlappingPariCache = new btDbvtBroadphase();

     btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

     //used to bring everything together?
     btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPariCache, solver, collisionConfig);

     dynamicsWorld->setGravity(btVector3(0, -3, 0));
     //initialized physics end
     //---------------------------------------------
     
     //---------------------------------------------
     // array of shapes
     btAlignedObjectArray<btCollisionShape*> collisionShapes;

     //-------creating collision ground-------
     //note to self: why do the scalars have a period at the end? signify specifity?
     {
          //btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
          btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(0.), btScalar(50.)));

          collisionShapes.push_back(groundShape);

          btTransform groundTransform;
          groundTransform.setIdentity();
          groundTransform.setOrigin(btVector3(0, groundOriginVal, 0));


          btScalar mass(0.);

          bool isDynamic = (mass != 0.f);

          btVector3 localInertia(0, 0, 0);
          if (isDynamic)
               groundShape->calculateLocalInertia(mass, localInertia);

          btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
          btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
          btRigidBody* body = new btRigidBody(rbInfo);

          dynamicsWorld->addRigidBody(body);
     }
     
     //-------creating collision object-------
     {
          btCollisionShape* colShape = new btSphereShape(btScalar(1.));
          collisionShapes.push_back(colShape);

          btTransform startTransform;
          startTransform.setIdentity();

          btScalar mass(1.f);

          bool isDynamic = (mass != 0.f);

          btVector3 localInertia(0, 0, 0);
          if (isDynamic)
               colShape->calculateLocalInertia(mass, localInertia);
          
          startTransform.setOrigin(btVector3(2, objOriginVal, 0));

          btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
          btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
          btRigidBody* body = new btRigidBody(rbInfo);

          dynamicsWorld->addRigidBody(body);
     }


     //---------------------------------------------

     Shader testShader("shader.vert", "shader.frag");
     //---------------------------------------------
     // declare models here
     Model obstacle("Resources/Objects/Obstacle/obsticle.obj");
     Model stickModel("Resources/Objects/Stick Man/Stick Test.obj");
     Model sphereModel("Resources/Objects/Basic Shapes/sphere/Sphere.obj");
     
     //Translation | Scaling | Rotaion
     GameObject obs(obstacle, glm::vec3(0.0f, groundOriginVal, 0.0f), glm::vec3(5.0f, 1.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f));
     //GameObject obs(obstacle,glm::vec3(0.0f, groundOriginVal, 0.0f), glm::vec3(5.0f, 1.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f));
     GameObject player(stickModel,glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.2f,0.2f,0.2f), glm::vec3(0.0f, 0.0f, 0.0f));
     GameObject sphere(sphereModel,glm::vec3(3.0f, objOriginVal, 0.0f), glm::vec3(1.0f, 1.0f, 1.0), glm::vec3(0.0f, 0.0f, 0.0f));

     
     //---------------------------------------------


     while (!glfwWindowShouldClose(window))
     {
          float currentFrame = static_cast<float>(glfwGetTime());
          deltaTime = currentFrame - lastFrame;
          lastFrame = currentFrame;

          processInput(window);
          glClearColor(0, 0, 0, 1.0f);
          glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

          //Physics Simulation start
          dynamicsWorld->stepSimulation(1.f / 60.f, 10);

          testShader.use();

          // view/projection transformations
          glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_Width / (float)SCR_Height, 0.1f, 100.0f);
          glm::mat4 view = camera.GetViewMatrix();
          testShader.setMat4("projection", projection);
          testShader.setMat4("view", view);

               for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
                    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
                    btRigidBody* body = btRigidBody::upcast(obj);
                    btTransform trans;
                    if (body && body->getMotionState())
                         body->getMotionState()->getWorldTransform(trans);
                    else
                         trans = obj->getWorldTransform();
                    printf("world pos object %d = %f,%f,%f\n", i, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));

                    //----Tresing tranformation----
                    if (1 == i)
                         sphere.setPosition(glm::vec3(float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ())));
                    //----Tresing tranformation----


               }
               cout << '\n';
          //Physics Simulation end

          //The Below is where all models that are to be rendered are to be placed
          player.render(testShader);
          obs.render(testShader);
          sphere.render(testShader);

          /*
          //Custom Controls
          if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
               sphere.setPosition(glm::vec3(camera.Position.x,camera.Position.y,camera.Position.z - 5.0f));
               sphere.render(testShader, sphereModel);
          }
          */


          glfwSwapBuffers(window);
          glfwPollEvents();
     }

     //Physics Objects Cleanup
     for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
          btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
          btRigidBody* body = btRigidBody::upcast(obj);
          if (body && body->getMotionState())
               delete body->getMotionState();
          dynamicsWorld->removeCollisionObject(obj);
          delete obj;
     }

     for (int j = 0; j < collisionShapes.size(); j++) {
          btCollisionShape* shape = collisionShapes[j];
          collisionShapes[j] = 0;
          delete shape;
     }

     delete dynamicsWorld;
     delete solver;
     delete overlappingPariCache;
     delete dispatcher;
     delete collisionConfig;
     collisionShapes.clear();
     //---------------------

   
     glfwTerminate();
     return 0;
}

void processInput(GLFWwindow* window)
{
     if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
     {
          glfwSetWindowShouldClose(window, true);
     }
     if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
     {
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
     }
     if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
     {
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
     }

     //Moves Camera Around
     if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
          camera.ProcessKeyboard(FORWARD, deltaTime);
     if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
          camera.ProcessKeyboard(BACKWARD, deltaTime);
     if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
          camera.ProcessKeyboard(LEFT, deltaTime);
     if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
          camera.ProcessKeyboard(RIGHT, deltaTime);

}


void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
     glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn) {
     float xpos = static_cast<float>(xposIn);
     float ypos = static_cast<float>(yposIn);

     if (firstMouse) {
          lastX = xpos;
          lastY = ypos;
          firstMouse = false;
     }

     float xoffset = xpos - lastX;
     float yoffset = lastY - ypos;
     
     lastX = xpos;
     lastY = ypos;

     camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
     camera.ProcessMouseScroll(static_cast<float>(yoffset));
}