#include "Planning.h"
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <sstream>
#include <list>
/****************************************/
/****************************************/

static const std::string QUEUEFILE        = "queue_";
static const std::string FILE_PREFIX      = "conv_";
static const std::string CHUNK_FILE_PREFIX= "chunk_";
static const std::string P2PFILE_PREFIX   = "queuep2p_";
static std::string POSTITIONFILE_PREFIX   = "pos_";
static std::string EFFECTIVENESS_PREFIX   = "effec_";
static std::string PERF_PREFIX   = "perf_";
static const Real        RAB_RANGE        = 7.0f;
static const UInt32 DATA_SIZE = 500;
// static std::string MSGFILE_PREFIX   = "msg_";
     
static const Real        FB_RADIUS        = 0.0704f;
static const Real        FB_AREA          = ARGOS_PI * Square(0.0704f);
static const std::string FB_CONTROLLER    = "khivbz";
static const UInt32      MAX_PLACE_TRIALS = 400;
static const UInt32      MAX_ROBOT_TRIALS = 400;

static const Real        WALL_THICKNESS   = 0.1;
static const Real        WALL_HEIGHT      = 0.3;
// static const Real        PLANE_RESOLUTION = 0.1;
static int fault_set = 0;
static const char free_space = '.';
static const char free_space_l = 'G';
static const char outofbound_space = '@';
static const char outofbound_space_l = 'O';
static const char Tree_space = 'T';
static const char Swap_space = 'S';
static const char Water_space = 'W';
static const char START_space = 'A';
static const char TARGET_space = 'X';
static const char WALL_space = 'Q';
static const Real TREE_RADIUS = 0.5f;
static const Real OUTOFBOUND_BOX_SIZE = 1.0f;
static const CRange<Real> CRANGE_START_POS(-1.5, 1.5);
static const CRange<Real> CRANGE_SPIRI_START_POS(-4.5,-2.5);
static const float OBSTACLE_SIDES1 = 1.1f;
static const float OBSTACLE_SIDES2 = 1.1f;
static const float FAILURE_PROBABILITY = 0.0005; // 0.00005 -> causes failour per 500 step



/****************************************/
/****************************************/

Planningloop::Planningloop() :
   m_bDone(false) {
}

/****************************************/
/****************************************/

Planningloop::~Planningloop() {
}

/****************************************/
/****************************************/

void Planningloop::Init(TConfigurationNode& t_tree) {
  LOG.Flush();
   LOGERR.Flush();
   try {

      /* Parse the configuration file */
      GetNodeAttribute(t_tree, "robots", unRobots);
      std::string OUT_FNAME; 
      GetNodeAttribute(t_tree, "out_file", OUT_FNAME);
      POSTITIONFILE_PREFIX +=OUT_FNAME;
      EFFECTIVENESS_PREFIX +=OUT_FNAME;
      PERF_PREFIX +=OUT_FNAME;


      GetNodeAttribute(t_tree, "map_file_name", m_map_file_name);
      int map_option = 0;
      GetNodeAttribute(t_tree, "map_option", map_option);
      if(map_option == 1){
         // Load the specifed .map file into the arena
         Load3DMapIntoArena(m_map_file_name);
      }
      else{
         LoadMapIntoArena(m_map_file_name);
      }

      std::string path; 
      GetNodeAttribute(t_tree, "path", path);

      float inter_cage_dist; 
      GetNodeAttribute(t_tree, "inter_cage_dist", inter_cage_dist);

      int random_seed; 
      GetNodeAttribute(t_tree, "random_seed_set", random_seed);
      int object_type;
      GetNodeAttribute(t_tree, "object_type", object_type);

      // Place the robots arounds the start point randomly
      CRange<Real> c_range_;
      if(unRobots <=50){
        CRange<Real> c_range_50(-6,-4);
        c_range_ = c_range_50;
      }  
      else{
        CRange<Real> c_range_100(-10,-6);
        c_range_ = c_range_100;
      }
      PlaceUniformly(unRobots,
                     DATA_SIZE,
                     c_range_);

      PlacePushedObject(object_type);
       for(int i=0; i< unRobots;++i) {
          /* Create a pointer to the current kh4 */
          CKheperaIVEntity& pcKB = *any_cast<CKheperaIVEntity *>(m_fbvec[i]);
          CConnectivityBuzzControllerKheperaIV* pcKBC = &dynamic_cast<CConnectivityBuzzControllerKheperaIV&>(pcKB.GetControllableEntity().GetController());
          m_vecControllers.push_back(pcKBC);
          buzzvm_t tBuzzVM = pcKBC->GetBuzzVM();
          // Update the number of robots 
          buzzvm_pushs(tBuzzVM, buzzvm_string_register(tBuzzVM, "total_bots", 1));
          buzzvm_pushi(tBuzzVM, unRobots);
          buzzvm_gstore(tBuzzVM);
          // Update the random seed.
          // Update the number of robots 
          buzzvm_pushs(tBuzzVM, buzzvm_string_register(tBuzzVM, "RANDOM_SEED_SET", 1));
          buzzvm_pushi(tBuzzVM, random_seed);
          buzzvm_gstore(tBuzzVM);
          // Update inter caging distance 
          buzzvm_pushs(tBuzzVM, buzzvm_string_register(tBuzzVM, "INTER_ROBOT_CAGING_DIS", 1));
          buzzvm_pushf(tBuzzVM, inter_cage_dist);
          buzzvm_gstore(tBuzzVM);
          if(path == "straight"){
            buzzvm_function_call(tBuzzVM, "load_streight_path", 0);
            /*Clear the nil value returned*/
            buzzvm_pop(tBuzzVM);
          }
          else if(path == "zigzac"){
            buzzvm_function_call(tBuzzVM, "load_zigzac_path", 0);
            /*Clear the nil value returned*/
            buzzvm_pop(tBuzzVM);
          } 
          else if(path == "straight_rot"){
            buzzvm_function_call(tBuzzVM, "load_streight_rot_path", 0);
            /*Clear the nil value returned*/
            buzzvm_pop(tBuzzVM);
          } 
          else{
            buzzvm_function_call(tBuzzVM, "load_default_path", 0);
            /*Clear the nil value returned*/
            buzzvm_pop(tBuzzVM);
          } 
          /* Append to list */
          m_buzz_ctrl.push_back(tBuzzVM);
       }
 
      /* Initialize the rest */
      Reset();
   
   }  
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/

void Planningloop::PostStep() {
  /* Get the object and find the pos of the object */
  std::stringstream os;
  os << "push_object";
  CEntity& c_entity = (GetSpace().GetEntity(os.str()));
  CBoxEntity* c_box_entity = (CBoxEntity*)&c_entity;
  m_posFile << std::endl;
  m_effecFile << std::endl;
  m_posFile << GetSpace().GetSimulationClock();
  m_effecFile << GetSpace().GetSimulationClock();
  // printf("Box pos  X %f, y %f\n",c_box_entity->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),c_box_entity->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
  for(int i=0; i< m_buzz_ctrl.size();++i){
    // Log position 
    /* At first you get the footbot object */
    CKheperaIVEntity& cEFootBot = *any_cast<CKheperaIVEntity *>(m_fbvec[i]);
    m_posFile << "," << i
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();


    buzzvm_t vm = m_buzz_ctrl[i];
    // Update the position of the pushed object on the robot.
    buzzvm_pushs(vm, buzzvm_string_register(vm, "Pushed_obj_pos", 1));
    buzzvm_pusht(vm);
    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 1));
    buzzvm_pushf(vm, c_box_entity->GetEmbodiedEntity().GetOriginAnchor().Position.GetX());
    buzzvm_tput(vm);
    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 1));
    buzzvm_pushf(vm, c_box_entity->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    buzzvm_tput(vm);
    buzzvm_gstore(vm);
    // log the effective pushers.
    buzzvm_pushs(vm, buzzvm_string_register(vm, "effective_pusher", 1));
    buzzvm_gload(vm);
    m_effecFile << "," << i << "," << buzzvm_stack_at(vm, 1)->i.value;
    buzzvm_pop(vm);
    // log the effective rotators.
    buzzvm_pushs(vm, buzzvm_string_register(vm, "effective_rotater", 1));
    buzzvm_gload(vm);
    m_effecFile << "," << buzzvm_stack_at(vm, 1)->i.value;
    buzzvm_pop(vm);
    // Log current pushing wp. 
    buzzvm_pushs(vm, buzzvm_string_register(vm, "TASK_NUM", 1));
    buzzvm_gload(vm);
    m_effecFile << "," << buzzvm_stack_at(vm, 1)->i.value;
    buzzvm_pop(vm);
    
    buzzvm_pushs(vm, buzzvm_string_register(vm, "EXPERIMENT_DONE", 1));
    buzzvm_gload(vm);
    if(buzzvm_stack_at(vm, 1)->i.value == 1){
      m_bDone = true;
    }
    buzzvm_pop(vm);
  }
  // Write the pushed object pos.
  CQuaternion& c_quat = c_box_entity->GetEmbodiedEntity().GetOriginAnchor().Orientation;
  CRadians cYaw, cPitch, cRoll;
  c_quat.ToEulerAngles(cYaw, cPitch, cRoll);
  m_posFile << "," << c_box_entity->GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
            << "," << c_box_entity->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
            << "," << cYaw.GetValue();

}

/****************************************/
/****************************************/

void Planningloop::Reset() {
   
   OpenFile(m_posFile,      POSTITIONFILE_PREFIX);
   OpenFile(m_effecFile,      EFFECTIVENESS_PREFIX);
   OpenFile(m_perfFile,      PERF_PREFIX);
}

/****************************************/
/****************************************/

void Planningloop::Destroy() {

   CloseFile(m_posFile);
   CloseFile(m_effecFile);
   CloseFile(m_perfFile);
}

/****************************************/
/****************************************/

bool Planningloop::IsExperimentFinished() {
   return m_bDone;
}


/****************************************/
/****************************************/

struct SFData {
   
   struct SEntry {
      UInt32 Conns;
      CVector3& Pos;
      SEntry(UInt32 un_conns,
             CVector3& c_pos) :
         Conns(un_conns),
         Pos(c_pos) {}
   };

   SFData() :
      TotConns(0),
      RNG(CRandom::CreateRNG("argos")) {}

   ~SFData() {
      while(!Data.empty()) {
         delete Data.front();
         Data.pop_front();
      }
   }

   void Insert(CKheperaIVEntity& c_entity) {
      /* Two connections to be added: entity <-> pivot */
      TotConns += 2;
      Data.push_back(
         new SEntry(1,
                    c_entity.GetEmbodiedEntity().
                    GetOriginAnchor().Position));
   }

   SEntry* Pick() {
      if(Data.size() > 1) {
         /* More than 1 element stored, look for the pivot */
         UInt32 x = RNG->Uniform(CRange<UInt32>(0, TotConns));
         UInt32 unSum = 0;
         std::list<SEntry*>::iterator it = Data.begin();
         while(it != Data.end() && unSum <= x) {
            unSum += (*it)->Conns;
            ++it;
         }
         if(it != Data.end()) {
            --it;
            return *it;
         }
         else {
            return Data.back();
         }
      }
      else if(Data.size() == 1) {
         /* One element stored, just return that one */
         return Data.front();
      }
      else THROW_ARGOSEXCEPTION("SFData::Pick(): empty structure");
   }

private:

   std::list<SEntry*> Data;
   UInt32 TotConns;
   CRandom::CRNG* RNG;
   
};





/****************************************/
/****************************************/

void Planningloop::OpenFile(std::ofstream& c_stream,
                      const std::string& str_prefix) {
   /* Make filename */
   std::string strFName = str_prefix + m_strOutFile;
   /* Close file and reopen it */
   CloseFile(c_stream);
   c_stream.open(strFName.c_str(),
                 std::ofstream::out | std::ofstream::trunc);
   if(c_stream.fail())
      THROW_ARGOSEXCEPTION("Error opening \"" << strFName << "\": " << strerror(errno));
}

/****************************************/
/****************************************/

void Planningloop::CloseFile(std::ofstream& c_stream) {
   if(c_stream.is_open()) c_stream.close();
}

/****************************************/
/****************************************/

void Planningloop::PlaceUniformly(UInt32 un_robots,
                            UInt32 un_data_size,
                            CRange<Real> c_area_range) {
  UInt32 unTrials;
  CKheperaIVEntity* pcFB;
  std::ostringstream cFBId;
  CVector3 cFBPos;
  CQuaternion cFBRot;
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
  /* For each robot */
  for (size_t i = 0; i < un_robots; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CKheperaIVEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add its controller to the list */
    // m_vecControllers.push_back(
    //     &dynamic_cast<CConnectivityBuzzControllerKheperaIV&>(
    //         pcFB->GetControllableEntity().GetController()));
    m_fbvec.push_back(pcFB);
    /* Try to place it in the arena */
    unTrials = 0;
    bool bDone;
    do {
      /* Choose a random position */
      ++unTrials;
      cFBPos.Set(pcRNG->Uniform(c_area_range),
                 pcRNG->Uniform(c_area_range),
                 0.0f);
      cFBRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                           CVector3::Z);
      bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
    } while (!bDone && unTrials <= MAX_PLACE_TRIALS);
    if (!bDone) {
      THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
    }
  }
}

void Planningloop::PlacePushedObject(int object_type){
  /*Object type enum 
    0 - square object of size (2,2) for 25 robots
    1 - square object of size (3.6,6) for 50 robots 
    2 - square object of size (7.2,12) for 100 robots
    3 - Cloud shape for caging tests
    4 - box_rotation shape for caging tests
    5 - clover shape for caging tests
  */
  if(object_type == 0){
    // height 0.000001
    // Mass of the object M = v * rho = 0.000004 cubic meter * 1.39 kilogram/cubic centimeter
    // Rho of plastic 1.39kilogram/cubic centimeter
    AddEntity( *new CBoxEntity("push_object",
                                CVector3(0,0,0),
                                CQuaternion(),
                                true,
                                CVector3(2, 2, 0.3),
                                5.56));
  }
  else if(object_type == 1){
    AddEntity( *new CBoxEntity("push_object",
                                CVector3(0,0,0),
                                CQuaternion(),
                                true,
                                CVector3(3.6, 6, 0.3),
                                30.024));
  }
  else if(object_type == 2){
    AddEntity( *new CBoxEntity("push_object",
                                CVector3(0,0,0),
                                CQuaternion(),
                                true,
                                CVector3(7.2, 12, 0.3),
                                120.096));
  }
  else if(object_type == 3){
    // Add cube
    AddEntity( *new CBoxEntity("push_object",
                                  CVector3(0,0,0),
                                  CQuaternion(),
                                  false,
                                  CVector3(2, 2, 0.3)));
    std::vector<float> pos_vec;
    pos_vec.push_back(-0.5);
    pos_vec.push_back(0.5);
    pos_vec.push_back(0.0);

    // Top cylinder    
    for(int i=0; i < pos_vec.size(); ++i){
      AddEntity(
      *new CCylinderEntity("col_"+std::to_string(i),
                           CVector3( 1, pos_vec[i], 0),
                           CQuaternion(),
                           false,
                           0.3,
                           0.3));  
    }
      

    // Top cylinder    
    for(int i=0; i < pos_vec.size(); ++i){
      AddEntity(
      *new CCylinderEntity("colb_"+std::to_string(i),
                           CVector3( -1, pos_vec[i], 0),
                           CQuaternion(),
                           false,
                           0.3,
                           0.3));  
    }

    // right cylinder  
    for(int i=0; i < pos_vec.size(); ++i){
      AddEntity(
      *new CCylinderEntity("colr_"+std::to_string(i),
                           CVector3(pos_vec[i], -1, 0),
                           CQuaternion(),
                           false,
                           0.3,
                           0.3));  
    }
 

    // left cylinder  
    for(int i=0; i < pos_vec.size(); ++i){
      AddEntity(
      *new CCylinderEntity("coll_"+std::to_string(i),
                           CVector3(pos_vec[i], 1, 0),
                           CQuaternion(),
                           false,
                           0.3,
                           0.3));  
    }
  }
  else if(object_type == 4){
    AddEntity( *new CBoxEntity("push_object",
                                CVector3(0,0,0),
                                CQuaternion(),
                                false,
                                CVector3(0.01, 0.01, 0.3)));
    for(int i=0; i < 8; ++i){
      CQuaternion m_orientation=CQuaternion();
      CVector3 m_box_pos = CVector3(1,0,0);
      m_orientation.FromEulerAngles(CRadians(0.785398*i),CRadians(0.0), CRadians(0.0));
      m_box_pos.Rotate(m_orientation);
      m_orientation.FromEulerAngles(CRadians(0.785398),CRadians(0.0), CRadians(0.0));
      AddEntity( *new CBoxEntity("push_object"+std::to_string(i),
                                m_box_pos,
                                m_orientation,
                                false,
                                CVector3(0.8, 0.8, 0.3)));
    }

  }
  else if(object_type == 4){
    AddEntity( *new CBoxEntity("push_object",
                                CVector3(0,0,0),
                                CQuaternion(),
                                false,
                                CVector3(0.01, 0.01, 0.3)));
    for(int i=0; i < 8; ++i){
      CQuaternion m_orientation=CQuaternion();
      CVector3 m_box_pos = CVector3(1,0,0);
      m_orientation.FromEulerAngles(CRadians(0.785398*i),CRadians(0.0), CRadians(0.0));
      m_box_pos.Rotate(m_orientation);
      m_orientation.FromEulerAngles(CRadians(0.785398),CRadians(0.0), CRadians(0.0));
      AddEntity( *new CBoxEntity("push_object"+std::to_string(i),
                                m_box_pos,
                                m_orientation,
                                false,
                                CVector3(0.8, 0.8, 0.3)));
    }
  }
  else if(object_type == 5){
    AddEntity( *new CBoxEntity("push_object",
                                CVector3(0,0,0),
                                CQuaternion(),
                                false,
                                CVector3(0.5, 0.5, 0.3)));
    for(int i=0; i < 4; ++i){
      CQuaternion m_orientation=CQuaternion();
      CVector3 m_box_pos = CVector3(0.7,0,0);
      m_orientation.FromEulerAngles(CRadians(1.5708*i),CRadians(0.0), CRadians(0.0));
      m_box_pos.Rotate(m_orientation);
      m_orientation.FromEulerAngles(CRadians(0.0), CRadians(0.0), CRadians(0.0));

      // AddEntity( *new CBoxEntity("push_object"+std::to_string(i),
      //                           m_box_pos,
      //                           m_orientation,
      //                           false,
      //                           CVector3(0.8, 0.8, 0.3)));

      AddEntity(
      *new CCylinderEntity("coll_"+std::to_string(i),
                           m_box_pos,
                           m_orientation,
                           false,
                           0.6,
                           0.3));
    }
  }
}
/****************************************/
/****************************************/

void Planningloop::LoadMapIntoArena(std::string m_map_file_name){
  std::string line;
  std::ifstream m_map_file (m_map_file_name);
  float length_offset = 0.3;
  float height_offset = 0.8;
  map_height = 0;
  map_length = 0;
  double half_map_length, half_map_height = 0;
  std::vector<std::vector<int>> v_map;
  int line_num = 0;
  if (m_map_file.is_open())
  {
    while ( getline (m_map_file,line) )
    { 
      if(line_num < 4){ // header
        if(line_num == 1){
           std::string str_height = line.substr (7,line.size());
           map_height = std::stoi(str_height) ;
           // std::cout << "height "<< map_height << '\n';
        }
        if(line_num == 2){
           std::string str_length = line.substr (6,line.size());
           map_length = std::stoi(str_length) ;
           // std::cout << "length "<< map_length << '\n';
        }
        if(line_num == 3){
          half_map_height = (map_height/2) * -1;
          half_map_length = (map_length/2) * -1;
          // std::cout << "half length "<< half_map_length << '\n';
          // std::cout << "half height "<< half_map_height << '\n';
        }
      }
      else{
      // std::cout << line << '\n';
        for(int i=0;i<line.size();i++){
          if(i == line.size()-1){
            // If it the first line then add a wall for the space before as well
            if(line_num == 4){
              AddEntity(
                *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i),
                                CVector3((line_num-4) + half_map_height+height_offset, i+half_map_length+length_offset, 0),
                                CQuaternion(),
                                false,
                                CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
            }
            //Normally add a border wall to the right hand side
            AddEntity(
                *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i+1),
                                CVector3((line_num-4) + half_map_height+height_offset, i+half_map_length+1+length_offset, 0),
                                CQuaternion(),
                                false,
                                CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
              // printf("Tree to : %f, %f \n", (line_num-4) + half_map_height, i+half_map_length+1);
          }
          else{
            if(line[i] == free_space || line[i] == free_space_l);
            else if(line[i] == Tree_space){
              // Add a column for trees
              // AddEntity(
              //   *new CCylinderEntity("col_"+std::to_string(line_num)+"_"+std::to_string(i),
              //                        CVector3( (line_num-3) + half_map_height, i+half_map_length, 0),
              //                        CQuaternion(),
              //                        false,
              //                        TREE_RADIUS,
              //                        WALL_HEIGHT));

              // Add a box for out of space entity
              AddEntity(
                *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i),
                                CVector3((line_num-4) + half_map_height+height_offset, i+half_map_length+length_offset, 0),
                                CQuaternion(),
                                false,
                                CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
              // printf("Tree to : %f, %f \n", (line_num-4) + half_map_height, i+half_map_length);
            }
            else if(line[i] == outofbound_space || line[i] == outofbound_space_l){
              // Add a box for out of space entity
              AddEntity(
                *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i),
                                CVector3((line_num-4) + half_map_height+height_offset, i+half_map_length+length_offset, 0),
                                CQuaternion(),
                                false,
                                CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
            }
            else if(line[i] == START_space){
              // Add a long thin column for start for debugging
              // AddEntity(
              //   *new CCylinderEntity("col_"+std::to_string(line_num)+"_"+std::to_string(i),
              //                        CVector3( (line_num-3) + half_map_height, i+half_map_length, 0),
              //                        CQuaternion(),
              //                        false,
              //                        0.1,
              //                        WALL_HEIGHT*4));
              // Store the start point
              Start_state = CVector2((line_num-4) + half_map_height, i+half_map_length);
              // std::cout << "Start state  "<< Start_state.GetX()<<" , "<< Start_state.GetY() << '\n';
            }
            else if(line[i] == TARGET_space){
              // Add a long thin column for end for debugging
              // AddEntity(
              //   *new CCylinderEntity("col_"+std::to_string(line_num)+"_"+std::to_string(i),
              //                        CVector3( (line_num-3) + half_map_height, i+half_map_length, 0),
              //                        CQuaternion(),
              //                        false,
              //                        0.1,
              //                        WALL_HEIGHT*1.5));
              // Store Goal state
              Goal_state = CVector2((line_num-4) + half_map_height, i+half_map_length);
              // std::cout << "Goal state  "<< Goal_state.GetX()<<" , "<< Goal_state.GetY() << '\n';

            }
          } 
        }
      }
      line_num +=1;
    }

    for(int i=0;i<map_length;i++){
      if(i == map_length-1){
        AddEntity(
            *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i),
                            CVector3((line_num-4) + half_map_height+height_offset, i+half_map_length+length_offset, 0),
                            CQuaternion(),
                            false,
                            CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
          // printf("last Tree to : %f, %f \n", (line_num-4) + half_map_height, i+half_map_length);

        AddEntity(
            *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i+1),
                            CVector3((line_num-4) + half_map_height+height_offset, i+half_map_length+1+length_offset, 0),
                            CQuaternion(),
                            false,
                            CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
          // printf("last Tree to : %f, %f \n", (line_num-4) + half_map_height, i+half_map_length+1);
      }
      else{
        AddEntity(
            *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i),
                            CVector3((line_num-4) + half_map_height+height_offset, i+half_map_length+length_offset, 0),
                            CQuaternion(),
                            false,
                            CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
          // printf("last Tree to : %f, %f \n", (line_num-4) + half_map_height, i+half_map_length);
      }
    }
    m_map_file.close();
  }

  else THROW_ARGOSEXCEPTION("Unable to open Map file"); 
}

/****************************************/
/****************************************/

void Planningloop::Load3DMapIntoArena(std::string m_map_file_name){
  std::string line;
  std::ifstream m_map_file (m_map_file_name);
  map_height = 0;
  map_length = 0;
  int map_planes = 0;
  double half_map_length, half_map_height = 0;
  int current_plane = 0;
  double plane_pos = (current_plane + 1) * PLANE_RESOLUTION;
  int map_end=0;
  std::string rectangle_name;
  std::vector<std::vector<int>> v_map;
  if (m_map_file.is_open())
  {
    int line_num = 0;
    while ( getline (m_map_file,line) )
    { 
      if(line_num < 4){ // header
        // std::cout << line << '\n';
        if(line_num == 1){
           std::string str_height = line.substr (7,line.size());
           map_height = std::stoi(str_height) ;
           // std::cout << "height "<< map_height << '\n';
        }
        if(line_num == 2){
           std::string str_length = line.substr (6,line.size());
           map_length = std::stoi(str_length);
           // std::cout << "length "<< map_length << '\n';
        }
        if(line_num == 3){
          half_map_height = (map_height/2.0) * -1.0;
          half_map_length = (map_length/2.0) * -1.0;
          std::string str_height = line.substr (7,line.size());
          map_planes = std::stoi(str_height);
          // std::cout << "half length "<< half_map_length << '\n';
          // std::cout << "half height "<< half_map_height << '\n';
        }
      }
      else{
        plane_pos = (current_plane + 1) * PLANE_RESOLUTION;
        std::string plane_str = line.substr(0,8);
        if(plane_str == "Rectagle"){
          rectangle_name = line.substr (9,line.size());
          // std::cout<<" Rectangle "<< rectangle_name<<std::endl;
          map_end = 1;
        }
        else if(map_end){
          std::size_t last_found = 1;
          std::size_t found = line.find_first_of(",");
          double rect_pos_size[6]={0};
          int pos_index=0;
          while (found!=std::string::npos)
          {
            std::string temp_pos = line.substr(last_found,found-last_found);
            rect_pos_size[pos_index] = std::stod(temp_pos);
            last_found = found+1;
            pos_index++;
            found=line.find_first_of(",",found+1);

          }
          found=line.find_first_of("\"",last_found);
          std::string temp_pos = line.substr(last_found,found-last_found);
          // std::cout<<"Rect X "<<rect_pos_size[0]+ half_map_height<<" Y "
          //   <<rect_pos_size[1]+half_map_length<<" Z "<<rect_pos_size[2]<<" half_map_height "
          //   <<half_map_height<<" half_map_length "<<half_map_length<<std::endl;
          rect_pos_size[pos_index] = std::stod(temp_pos);
          AddEntity(
                *new CBoxEntity("Rectangle_"+rectangle_name,
                                CVector3(rect_pos_size[0]+ half_map_height,rect_pos_size[1]+half_map_length,rect_pos_size[2]),
                                CQuaternion(),
                                false,
                                CVector3(rect_pos_size[3],rect_pos_size[4],rect_pos_size[5])));

        }
        else {
          // std::cout<<" Plane "<<current_plane<<std::endl;
          // std::cout << line<<"\n"<< " plane_num "<<current_plane <<" line_num "<<line_num<<" plane pos "<<plane_pos << "\n";
          // std::cout<<" line "<< line_num-4<<std::endl;
          for(int i=0;i<line.size();i++){

            if(line[i] == free_space || line[i] == free_space_l);
            else if(line[i] == Tree_space){
              // Add a column for trees
              AddEntity(
                *new CCylinderEntity("col_"+std::to_string(line_num)+"_"+std::to_string(i)+'_'+std::to_string(current_plane),
                                     CVector3( (line_num-4) + half_map_height, i+half_map_length, 0),
                                     CQuaternion(),
                                     false,
                                     TREE_RADIUS,
                                     WALL_HEIGHT));
            }
            else if(line[i] == outofbound_space || line[i] == outofbound_space_l){
              // Add a box for out of space entity
              AddEntity(
                *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i)+'_'+std::to_string(current_plane),
                                CVector3((line_num-4) + half_map_height, i+half_map_length, 0),
                                CQuaternion(),
                                false,
                                CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, WALL_HEIGHT)));
            }
            // else if(line[i] == WALL_space){
            //   // Add a box for out of space entity
            //   AddEntity(
            //     *new CBoxEntity("box"+std::to_string(line_num)+"_"+std::to_string(i)+'_'+std::to_string(current_plane),
            //                     CVector3((line_num-5) + half_map_height, i+half_map_length, plane_pos),
            //                     CQuaternion(),
            //                     false,
            //                     CVector3(OUTOFBOUND_BOX_SIZE, OUTOFBOUND_BOX_SIZE, PLANE_RESOLUTION)));
            // }
            else if(line[i] == START_space && current_plane == 0){
              // Add a long thin column for start for debugging
              // AddEntity(
              //   *new CCylinderEntity("col_"+std::to_string(line_num)+"_"+std::to_string(i),
              //                        CVector3( (line_num-3) + half_map_height, i+half_map_length, 0),
              //                        CQuaternion(),
              //                        false,
              //                        0.1,
              //                        WALL_HEIGHT*4));
              // Store the start point
              Start_state = CVector2((line_num-4) + half_map_height, i+half_map_length);
              std::cout << "Start state  "<< Start_state.GetX()<<" , "<< Start_state.GetY() << '\n';
            }
            else if(line[i] == TARGET_space && current_plane == 0){
            //   // Add a long thin column for end for debugging
            // AddEntity(
            //   *new CCylinderEntity("col_"+std::to_string(line_num)+"_"+std::to_string(i),
            //                        CVector3( (line_num-4) + half_map_height, i+half_map_length, 0),
            //                        CQuaternion(),
            //                        false,
            //                        0.1,
            //                        WALL_HEIGHT*1.5));
              // Store Goal state
              Goal_state = CVector2((line_num-4) + half_map_height, i+half_map_length);
              std::cout << "Goal state  "<< Goal_state.GetX()<<" , "<< Goal_state.GetY() << '\n';

            }
          }
        }
      }
      line_num +=1;
    }
    m_map_file.close();
  }

  else THROW_ARGOSEXCEPTION("Unable to open Map file"); 

}

REGISTER_LOOP_FUNCTIONS(Planningloop, "Planning");
