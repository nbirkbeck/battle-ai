#ifndef GL_OGRE_WINDOW
#define GL_OGRE_WINDOW

#include <iostream>
#include "glwindow.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreLight.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreFrameListener.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreWindowEventUtilities.h>

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

class GLOgreWindow : public InteractiveCameraView,
  public Ogre::WindowEventListener,  public Ogre::FrameListener {
public:
  GLOgreWindow() : keyboard_(nullptr), mouse_(nullptr)
  {
    using namespace Ogre;
    bdown_ = 0;
    memset(keydown_, 0, sizeof(keydown_));

    root = new Ogre::Root("plugins.cfg");

    //if(!(root->restoreConfig() || root->showConfigDialog()))
    //  return -1;
    
    auto* rs = root->getRenderSystemByName("OpenGL Rendering Subsystem");
        
    root->setRenderSystem(rs);
    rs->setConfigOption("Full Screen", "No");
    window_ = root->initialise(true);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/tmp/agent", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/usr/share/OGRE/media/models", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/usr/share/OGRE/media", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/usr/share/OGRE/media/materials", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/usr/share/OGRE/media/materials/textures", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/usr/share/OGRE/media/materials/programs", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/usr/share/OGRE/media/materials/scripts", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/usr/share/OGRE/media/materials/programs/GLSL", "FileSystem");

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/birkbeck/Desktop/BattleAI/levels", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/birkbeck/Desktop/BattleAI/models", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/birkbeck/Desktop/BattleAI", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    
    OIS::InputManager* mInputManager;
    OIS::ParamList pl;
    size_t winHandle = 0;
    std::ostringstream winHandleStr;  
    window_->getCustomAttribute("WINDOW", &winHandle);
    winHandleStr << winHandle;
    pl.insert(std::make_pair("WINDOW", winHandleStr.str()));
    pl.insert(std::make_pair("x11_keyboard_grab", "false"));
    pl.insert(std::make_pair("x11_mouse_grab", "false"));
    
    mInputManager = OIS::InputManager::createInputSystem( pl );
    
    keyboard_ = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, false ));
    mouse_ = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, false ));
    Ogre::WindowEventUtilities::addWindowEventListener(window_, this);
    root->addFrameListener(this);


    // Setup the scene
    scnMgr = root->createSceneManager(Ogre::ST_GENERIC);
    scnMgr->setAmbientLight(Ogre::ColourValue(.2, .2, .2));


    camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    Camera* cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(0.5);
    cam->setAspectRatio(1920/1080.0);
    camNode->attachObject(cam);

    Viewport* vp = window_->addViewport(cam);
    vp->setBackgroundColour(ColourValue(0.7, 0.7, 0.7));
    
    SyncCamera();

    Entity* entity = scnMgr->createEntity("world.textpb.mesh");
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entity);

    Light* light = scnMgr->createLight("MainLight");
    light->setPosition(10, 100, 10);
    light->setDiffuseColour(0.4, 0.4, 0.4);
    light->setSpecularColour(0.0, 0.0, 0.0);
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(light);

    light = scnMgr->createLight("MainLight2");
    light->setPosition(0, 100, -20);
    light->setDiffuseColour(0.2, 0.2, 0.2);
    light->setSpecularColour(0.0, 0.0, 0.0);
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(light);

    light = scnMgr->createLight("MainLight3");
    light->setPosition(20, 60, 20);
    light->setDiffuseColour(0.2, 0.2, 0.2);
    light->setSpecularColour(0.0, 0.0, 0.0);
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(light);

    light = scnMgr->createLight("MainLight4");
    light->setType(Light::LT_DIRECTIONAL);
    light->setCastShadows(false);
    light->setDirection(-100, 10, -100);
    light->setDiffuseColour(0.5, 0.5, 0.5);
    light->setSpecularColour(0.0, 0.0, 0.0);
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(light);

    light = scnMgr->createLight("MainLight5");
    light->setType(Light::LT_DIRECTIONAL);
    light->setCastShadows(false);
    light->setDirection(100, 10, 100);
    light->setDiffuseColour(0.5, 0.5, 0.5);
    light->setSpecularColour(0.0, 0.0, 0.0);
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(light);

    
    scnMgr->setShadowTechnique(ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);
  }

  // Ogre::WindowEventListener
  void windowResized(Ogre::RenderWindow* rw) override {
    std::cerr << "Window resized.\n";
    //reshape();
  }

  void windowClosed(Ogre::RenderWindow* rw) override {
    std::cerr << "Window closed.\n";
    //closing();
  }
  virtual bool mouse(int button, bool down, int x, int y){
    std::cout << "Mouse:" << button << " down:" << down
              << " " << x << ", " << y << std::endl;
        
    return InteractiveCameraView::mouse(button, down, x, y);
  }

  virtual void motion(int x, int y){
    InteractiveCameraView::motion(x, y);
  }
  void UpdateMouse() {
    mouse_->capture();
    OIS::MouseState mouse_state = mouse_->getMouseState();
    int bdown = 0;
    if (mouse_state.buttonDown(OIS::MB_Left)) {
      bdown = 1;
    }
    if (mouse_state.buttonDown(OIS::MB_Middle)) {
      bdown = 2;
    }
    if (mouse_state.buttonDown(OIS::MB_Right)) {
      bdown = 3;
    }
    if (!bdown_ && bdown > 0) {
      mouse(bdown - 1, true, mouse_state.X.abs, mouse_state.Y.abs);
    }
    if ((bdown_ > 0) && (bdown == 0)) {
      mouse(bdown_ - 1, false, mouse_state.X.abs, mouse_state.Y.abs);
    }
    if (last_pos_[0] != mouse_state.X.abs || last_pos_[1] != mouse_state.Y.abs) {
      motion(mouse_state.X.abs, mouse_state.Y.abs);
    }
    last_pos_[0] = mouse_state.X.abs;
    last_pos_[1] = mouse_state.Y.abs;
    bdown_ = bdown;
  }

  void UpdateKeyboard() {
    keyboard_->capture();
    shift_down_ = false;
    for (int i = 1; i <= 0xED; ++i) {
      if (i == OIS::KC_RSHIFT ||
          i == OIS::KC_LSHIFT) {
        shift_down_ |= keyboard_->isKeyDown((OIS::KeyCode)i);
        continue;
      }
          
      if (keyboard_->isKeyDown((OIS::KeyCode)i)) {
        if (!keydown_[i]) {
          keyboard(TranslateKey(i), 0, 0);
        }
        keydown_[i] = 1;
      } else {
        if (keydown_[i]) {
          keyboardUp(TranslateKey(i), 0, 0);
        }
        keydown_[i] = 0;
      }
    }
  }
  void SyncCamera() {
    camNode->setPosition(cpos.x, cpos.y, cpos.z);
    camNode->setOrientation(cquat.a, cquat.v.x, cquat.v.y, cquat.v.z);
  }

  virtual void UpdateScene() {
  }
  
  bool frameRenderingQueued(const Ogre::FrameEvent& evt) {
    UpdateMouse();
    UpdateKeyboard();
    UpdateScene();
    SyncCamera();
    return true;
  }

  virtual bool keyboard(unsigned char c, int x, int y) {
    return false;
  }
  virtual bool keyboardUp(unsigned char c, int x, int y) {
    return false;
  }
  int width() override { return 1920; }
  int height() override { return 1920; }
  bool shiftActive() override {
    return shift_down_;
  }

  unsigned char TranslateKey(int c) {
    unsigned char mapping[] = {
                               0, // KC_UNASSIGNED  = 0x00,
                               0, // KC_ESCAPE      = 0x01,
                               '1', // KC_1           = 0x02,
                               '2', // KC_2           = 0x03,
                               '3', // KC_3           = 0x04,
                               '4', // KC_4           = 0x05,
                               '5', // nKC_5           = 0x06,
                               '6', // KC_6           = 0x07,
                               '7', // KC_7           = 0x08,
                               '8', // KC_8           = 0x09,
                               '9', // KC_9           = 0x0A,
                               '0', // KC_0           = 0x0B,
                               '-', // KC_MINUS       = 0x0C,    // - on main keyboard
                               '=', // KC_EQUALS      = 0x0D,
                               0,  // KC_BACK        = 0x0E,    // backspace
                               0,  // KC_TAB         = 0x0F,
                               'q', // KC_Q           = 0x10,
                               'w', // KC_W           = 0x11,
                               'e', // KC_E           = 0x12,
                               'r', // KC_R           = 0x13,
                               't', // KC_T           = 0x14,
                               'y', // KC_Y           = 0x15,
                               'u', // KC_U           = 0x16,
                               'i', // KC_I           = 0x17,
                               'o', // KC_O           = 0x18,
                               'p', // KC_P           = 0x19,
                               '[', // KC_LBRACKET    = 0x1A,
                               ']', // nKC_RBRACKET    = 0x1B,
                               '\n', // KC_RETURN      = 0x1C,    // Enter on main keyboard
                               0, // KC_LCONTROL    = 0x1D,
                               'a', // KC_A           = 0x1E,
                               's', // KC_S           = 0x1F,
                               'd', // KC_D           = 0x20,
                               'f', // KC_F           = 0x21,
                               'g', // KC_G           = 0x22,
                               'h', // KC_H           = 0x23,
                               'j', // KC_J           = 0x24,
                               'k', // KC_K           = 0x25,
                               'l', // KC_L           = 0x26,
                               ';', // KC_SEMICOLON   = 0x27,
                               '\'', // KC_APOSTROPHE  = 0x28,
                               0, // nKC_GRAVE       = 0x29,    // accent
                               0, // KC_LSHIFT      = 0x2A,
                               '\\', /// KC_BACKSLASH   = 0x2B,
                               'z', // KC_Z           = 0x2C,
                               'x', // KC_X           = 0x2D,
                               'c', // KC_C           = 0x2E,
                               'v', // nKC_V           = 0x2F,
                               'b', // nKC_B           = 0x30,
                               'n', // KC_N           = 0x31,
                               'm', // 		KC_M           = 0x32,
                               ',', //KC_COMMA       = 0x33,
                               '.', // nKC_PERIOD      = 0x34,    // . on main keyboard
                               '/', // nKC_SLASH       = 0x35,    // / on main keyboard
                               0,  // KC_RSHIFT      = 0x36,
                               '*',  // KC_MULTIPLY    = 0x37,    // * on numeric keypad
                               0, // KC_LMENU       = 0x38,    // left Alt
                               ' ', // KC_SPACE       = 0x39,

    };
    if (c < (int)sizeof(mapping)) return mapping[c];
    return 0;
  }

  bool loop() {
    while (true) {
      Ogre::WindowEventUtilities::messagePump();
      
      if(window_->isClosed()) return false;
      
      if(!root->renderOneFrame()) return false;
    }
    return true;
  }

 protected:
  OIS::Keyboard* keyboard_;
  OIS::Mouse* mouse_;
  bool keydown_[256];
  int last_pos_[2];
  int bdown_;
  
  bool shift_down_;
  Ogre::RenderWindow* window_;
  Ogre::Root* root;
  Ogre::SceneNode* camNode;
  Ogre::SceneManager* scnMgr;
};


#endif
