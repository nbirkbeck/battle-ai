#ifndef OGRE_WIN
#define OGRE_WIN

#include "src/agent/user_agent.h"
#include "src/world.h"
#include "src/ui/utigl/ogre_window.h"

#include <OGRE/OgrePrerequisites.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>

class OgreWin : public GLOgreWindow {
public:
  OgreWin(const std::string& level_mesh) : GLOgreWindow() {
    level_mesh_ = level_mesh;
    InitOverlay();
  }
  
  void SetWorld(World* world) {
    if (!world_) {
      world_ = world;
      InitScene(level_mesh_);
    }
  }

  void InitOverlay() {
    using Ogre::OverlayManager;
    overlay_ = OverlayManager::getSingleton().create("InfoOverlay");

    overlay_->setZOrder(10);
    overlay_container_ = (Ogre::OverlayContainer*)
      Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "InfoOverlayPanel");
    overlay_container_->setDimensions(1.0, 1.0);
    overlay_container_->setPosition(0.0, 0.0);
    
    overlay_->add2D(overlay_container_);
    overlay_->show();
  } 

  void InitScene(const std::string& level_mesh) {
    auto* level_entity = scnMgr->createEntity(level_mesh);
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(level_entity);

    int agent_index = 0;
    for (const auto& agent: world_->agents()) {
      char name[64];
      snprintf(name, sizeof(name), "Agent%d", agent_index); 
      (void)agent;
      
      Ogre::Entity* body_entity = scnMgr->createEntity("Body.mesh");
      Ogre::SceneNode* body_node = scnMgr->getRootSceneNode()->createChildSceneNode(name);
      body_node->attachObject(body_entity);

      Ogre::Entity* head_entity = scnMgr->createEntity("Head.mesh");
      body_node->createChildSceneNode(std::string(name) + ".Head")->attachObject(head_entity);

      Ogre::Entity* right_wheel = scnMgr->createEntity("RightWheel.mesh");
      body_node->createChildSceneNode(std::string(name) + ".RightWheel")->attachObject(right_wheel);

      Ogre::Entity* left_wheel = scnMgr->createEntity("LeftWheel.mesh");
      body_node->createChildSceneNode(std::string(name) + ".LeftWheel")->attachObject(left_wheel);

      agents_.push_back(body_node);
      agent_index++;
    }

    for (const auto& pup : world_->power_ups()) {
      Ogre::Entity* entity;
      if (pup->type() == state::PowerUp::POWER_UP_HEALTH) {
        entity = scnMgr->createEntity("HealthPowerup.mesh");
        LOG(INFO) << "Has skeleton:" << entity->hasSkeleton();
        auto* state = entity->getAnimationState("ArmatureAction.001");
        state->setEnabled(true);
        state->setLoop(true);
        state->setTimePosition(0);
        state->setLength(10);
      } else {
        entity = scnMgr->createEntity("ArmorPowerup.mesh");
      }

      Ogre::SceneNode* node = scnMgr->getRootSceneNode()->createChildSceneNode();
      node->attachObject(entity);
      node->setPosition(pup->pos().x, pup->pos().y, pup->pos().z);
      power_ups_.push_back(node);
    }


    UpdateScene();
  }

  void UpdateOverlayElements() {
    using Ogre::OverlayManager;
    for (const auto element : overlay_elements_) {
      overlay_container_->_removeChild(element);
      OverlayManager::getSingleton().destroyOverlayElement(element);
    }
    overlay_elements_.clear();
    
    for (int i = 0; i < (int)world_->agents().size(); ++i) {
      const auto& agent = world_->agents()[i];
      const Ogre::Vector3 p(agent->pos().x,  agent->pos().y + 1.8,  agent->pos().z);
      const auto pt = camera_->getProjectionMatrix() * (camera_->getViewMatrix() * p);
      const double x = (pt.x / 2) + 0.5f;
      const double y =  1 - ((pt.y / 2) + 0.5f);
      char name[1024];
      snprintf(name, sizeof(name), "TextArea=%d", i);

      Ogre::OverlayElement *l = OverlayManager::getSingleton().createOverlayElement("TextArea", name);
      char str[1024];
      snprintf(str, sizeof(str), "Index=%d\nHealth=%g\nArmor=%g", i,
               agent->health(), agent->armor());
      l->setCaption(str);
      l->setPosition(x, y);
      l->setDimensions(1, 1);  // center text in label and its position
      l->setParameter("font_name", "SdkTrays/Value");
      l->setParameter("char_height", "0.03f");
      l->setColour(Ogre::ColourValue(i == 0, i == 1, 0.0));
      
      overlay_container_->addChild(l);
      overlay_elements_.push_back(l);
    }

    
    Ogre::OverlayElement *l = OverlayManager::getSingleton().createOverlayElement("TextArea", "Stats");
    l->setCaption(info_string_);
    l->setPosition(0.0, 0.1);
    l->setDimensions(1, 1);  // center text in label and its position
    l->setParameter("font_name", "SdkTrays/Value");
    l->setParameter("char_height", "0.03f");
    l->setColour(Ogre::ColourValue(1, 1, 1, 1));
    overlay_container_->addChild(l);
    overlay_elements_.push_back(l);
  }
  
  void UpdateScene() {
    if (!world_) return;
    if (animating_) {
      world_->Step(1.0/240);
    }
    if (animating_ && first_person_ && user_agent_) {
      auto dir = user_agent_->quat().rotate(nacb::Vec3d(0, 0, 1));
      cpos = user_agent_->pos();
      cpos.y = 0;
      cpos += nacb::Vec3d(0, 1.5, 0) - dir * 0.25;
      cquat = user_agent_->quat() *  nacb::Quaternion::rod(nacb::Vec3d(0, M_PI, 0));
      SyncCamera();
    }

    // Remove overlay
    UpdateOverlayElements();
    
    for (int i = 0; i < (int)world_->agents().size(); ++i) {
      agents_[i]->setVisible((i > 0) || !first_person_);
      
      const auto& agent = world_->agents()[i];
      agents_[i]->setPosition(world_->agents()[i]->pos().x,
                              0,
                              world_->agents()[i]->pos().z);

      const nacb::Vec3d d = agent->pos() - agent->last_pos();
      nacb::Quaternion q;
      if (d.len()) {
        q = nacb::Quaternion::rod(nacb::Vec3d(0, atan2(d.x, d.z), 0));
        agents_[i]->setOrientation(q.a, q.v.x, q.v.y, q.v.z);
      } else {
        const auto& quat = agents_[i]->getOrientation();
        q.a = quat.w;
        q.v.x = quat.x;
        q.v.y = quat.y;
        q.v.z = quat.z;
        q.normalize();
      }
      nacb::Vec3d head_d = agent->quat().rotate(nacb::Vec3d(0, 0, 1));
      const double a2 = atan2(head_d.x, head_d.z);
      
      const nacb::Quaternion q2 = q.conj() * nacb::Quaternion::rod(nacb::Vec3d(0, a2, 0));
      agents_[i]->getChild(agents_[i]->getName() + ".Head")->setOrientation(q2.a, q2.v.x, q2.v.y, q2.v.z);
    }

    for (int i = 0; i < (int)world_->power_ups().size(); ++i) {
      Ogre::Entity* entity = (Ogre::Entity*)power_ups_[i]->getAttachedObject(0);
      if (entity->hasAnimationState("ArmatureAction.001")) {
        entity->getAnimationState("ArmatureAction.001")->addTime(1.0/240.0);
      }

      power_ups_[i]->setVisible(world_->power_ups()[i]->IsActive());
    }

    for (const auto& p: projectiles_) {
      scnMgr->getRootSceneNode()->removeChild(p);
      scnMgr->destroySceneNode(p);
    }

    if (world_->projectiles().size()) {
      std::cout << "Num projectiles:" <<
        world_->projectiles().size() << " " <<
        projectiles_.size() << 
        std::endl;
    }
    projectiles_.clear();

    for (const auto& proj : world_->projectiles()) {
      Ogre::Entity* entity;
      entity = scnMgr->createEntity("Bullet.mesh");
      
      Ogre::SceneNode* node = scnMgr->getRootSceneNode()->createChildSceneNode();
      node->attachObject(entity);

      const double theta = atan2(proj.v().x, proj.v().z);
      const nacb::Quaternion q = nacb::Quaternion::rod(nacb::Vec3d(0, theta, 0));
      node->setPosition(proj.p().x, proj.p().y, proj.p().z);
      node->setOrientation(q.a, q.v.x, q.v.y, q.v.z);
      
      projectiles_.push_back(node);
    }
  }

  bool keyboardUp(unsigned char c, int x, int y) override {
    return user_agent_ ? user_agent_->KeyboardUp(c, x, y) : true;
  }

  void motion(int x, int y) {
    if (bdown == 1) {
      double dx = x - mpos[0], dy = y - mpos[1];
      dx /= height();
      dy /= height();
      if (user_agent_) user_agent_->Rotate(-dx);
    }
    GLOgreWindow::motion(x, y);
  }

  bool keyboard(unsigned char c, int x, int y) override {
    switch (c) {
    case '1':
      first_person_ = !first_person_;
      break;
    case '0':
      animating_ = !animating_;
      break;
    }
    if (user_agent_) {
      return user_agent_->KeyboardDown(c, x, y);
    }
    return true;
  }

  void SetUserAgent(UserAgent* user_agent) {
    user_agent_ = user_agent;
  }

  void SetInfoString(const std::string str) {
    info_string_ = str;
  }

  UserAgent* user_agent_ = nullptr;
  std::vector<Ogre::SceneNode*> agents_;
  std::vector<Ogre::SceneNode*> power_ups_;
  std::vector<Ogre::SceneNode*> projectiles_;

  Ogre::Overlay *overlay_;
  Ogre::OverlayContainer *overlay_container_;
  std::vector<Ogre::OverlayElement*> overlay_elements_;
  
  std::string level_mesh_;
  std::string info_string_;
  World* world_ = nullptr;
  bool animating_ = false;
  bool first_person_ = false;
};

#endif  // OGRE_WIN
