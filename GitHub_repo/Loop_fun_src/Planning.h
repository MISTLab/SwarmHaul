#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/buzz_controller_kheperaiv.h>
#include<../Hooks_src/controller.h>
#include <buzz/argos/buzz_controller_spiri.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/robots/spiri/simulator/spiri_entity.h>
#include <fstream>
#include <string>
#include <cerrno>

using namespace argos;

class Planningloop : public CLoopFunctions {

public:

   Planningloop();
   virtual ~Planningloop();

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void Reset();
   virtual void Destroy();
   virtual bool IsExperimentFinished();

private:

   void LoadMapIntoArena(std::string m_map_file_name);
   void Load3DMapIntoArena(std::string m_map_file_name);
   
   void PlaceUniformly(UInt32 un_robots,
                       UInt32 un_data_size,
                       CRange<Real> c_range_);
   
   void OpenFile(std::ofstream& c_stream,
                 const std::string& str_prefix);
   void CloseFile(std::ofstream& c_stream);



   class Rectangle_t
{
public:
  /**
   * @brief Create a rectangle using two corners
   * 
   * @param lx Bottom Left X coordinate
   * @param ly Bottom Left Y coordinate
   * @param hx Top Right X coordinate
   * @param hy Top Right Y coordinate
   */
  Rectangle_t(double lx,double ly,double hx,double hy)
  {
    low_x = lx;
    low_y = ly;
    high_x = hx;
    high_y = hy;
  }
  /**
   * @brief Create a rectangle with center position and dimensions.
   * 
   * @param pos_x Center X coordinate.
   * @param pos_y Center Y coordinate.
   * @param dim_x X Dimension
   * @param dim_y Y Dimension
   * @param value Just a flag to denote which constructor is used.
   */
  Rectangle_t(double pos_x,double pos_y,double dim_x,double dim_y,bool value)
  {
    low_x = pos_x-dim_x/2;
    low_y = pos_y-dim_y/2;
    high_x = pos_x+dim_x/2;
    high_y = pos_y+dim_y/2;
  }

  std::vector<double> get_pos_dim(){
    std::vector<double> v;
    //X_pos
    v.push_back( high_x -((high_x-low_x)/2) ); 
    //Y_Pos
    v.push_back(high_y -((high_y-low_y)/2));
    // X_dim
    v.push_back( std::fabs((high_x-low_x)) );
    // Y_dim
    v.push_back( std::fabs((high_y-low_y)) );
    return v;
  }

  std::vector<double> get_hl(){
    std::vector<double> v;
    //X_pos
    v.push_back( high_x  ); 
    //Y_Pos
    v.push_back(high_y );
    // X_dim
    v.push_back( low_x );
    // Y_dim
    v.push_back(low_y);
    return v;
  }

  int find_coincide(Rectangle_t& other){
    if( (other.high_x -((other.high_x-other.low_x)/2)) > low_x && 
     (other.high_x -((other.high_x-other.low_x)/2)) < high_x && 
     (other.high_y -((other.high_y-other.low_y)/2)) > low_y && 
     (other.high_y -((other.high_y-other.low_y)/2)) < high_y ) return 1;
    else return 0;

  }

  double get_area(){
    return std::fabs((high_x-low_x))*std::fabs((high_y-low_y));
  }
  double low_x;
  double low_y;
  double high_x;
  double high_y;
};


private:

   std::string m_strOutFile;
   std::string m_map_file_name;
   double map_height;
   double map_length;
   bool m_bDone;

   std::ofstream m_posFile;
   std::ofstream m_effecFile;
   std::ofstream m_perfFile;

   std::vector<CConnectivityBuzzControllerKheperaIV*> m_vecControllers;
   std::vector<CKheperaIVEntity*> m_fbvec;
   std::vector<buzzvm_t> m_buzz_ctrl;

   
   CVector2 Start_state;
   CVector2 Goal_state;
   UInt32 unRobots;

};

