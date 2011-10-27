#include "hre_random_alg.h"
#include "xml/hri_cfg_file.hxx"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <fstream>

std::string node_path=std::string(getenv("IRI_ROS_STACK_PATH"))+std::string("/tibi_dabo_robot/hre_random_node/");
std::string motion_path="xml/motion/";
std::string speech_path="xml/speech/";
std::string home_seq=motion_path + "home.xml";

HreRandomAlgorithm::HreRandomAlgorithm()
{ 
  //setDriverId(driver string id);
  if(chdir(node_path.c_str())!=0)
    std::cout << "error changing the directory" << std::endl;
  // list all the existing XML files
  this->scan_XML_files();

  this->desired_home_interval=3;
  this->desired_speech_interval=1;
  this->desired_repeat_interval=3;

  srand(time(NULL));
}

void HreRandomAlgorithm::scan_XML_files(void)
{
  std::fstream speech_file;
  std::string full_path;
  char *text_data=NULL;
  struct dirent *dp;
  struct stat st;
  DIR *dir;

  // check wether the config XML files folder exists or not
  if(stat(speech_path.c_str(),&st)==0)// the folder exists
  {
    full_path=node_path + speech_path;
    dir=opendir(full_path.c_str());
    while((dp=readdir(dir)) != NULL)
    {
      if(strstr(dp->d_name,".txt")!=NULL)
      {
        // open the file
        full_path=speech_path + dp->d_name;
        speech_file.open(full_path.c_str(),std::fstream::in);
        stat(dp->d_name,&st);// get information about the file 
        text_data=new char[st.st_size];
        speech_file.get(text_data,st.st_size);
        this->speech_files.push_back(text_data);
        delete[] text_data;
        speech_file.close();
      }
    }
    closedir(dir);
  }
  std::cout << "Found " << this->speech_files.size() << " speech files" << std::endl;
  if(stat(motion_path.c_str(),&st)==0)// the folder exists
  {
    full_path=node_path + motion_path;
    dir=opendir(full_path.c_str());
    while((dp=readdir(dir)) != NULL)
    {
      if(strstr(dp->d_name,".xml")!=NULL)
        this->motion_seq_files.push_back(motion_path + dp->d_name);
    }
    closedir(dir);
  }
  std::cout << "Found " << this->motion_seq_files.size() << " motion sequence files" << std::endl;
}

HreRandomAlgorithm::~HreRandomAlgorithm()
{
}

void HreRandomAlgorithm::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  this->desired_home_interval=new_cfg.home_interval;
  this->desired_speech_interval=new_cfg.speech_interval;
  this->desired_repeat_interval=new_cfg.repeat_interval;
  
  this->unlock();
}

int HreRandomAlgorithm::random_seq(void)
{
  static std::vector<bool> tracked_seq(this->desired_repeat_interval,-1);
  bool repeated = true;
  int rnd_seq,i=0;

  rnd_seq=(int)(((float)rand()/(float)RAND_MAX)*(this->motion_seq_files.size()-1));

  while(repeated)
  {
    repeated = false;
    for(i=0; i<this->desired_repeat_interval; i++)
    {
      if(rnd_seq == tracked_seq[i])
        repeated = true;
    }
    if(repeated)
      rnd_seq=(int)(((float)rand()/(float)RAND_MAX)*(this->motion_seq_files.size()-1));
  }
  //track lasts sequences
  for(i=1;i<this->desired_repeat_interval;i++)
    tracked_seq[i-1]=tracked_seq[i];
  tracked_seq[2]=rnd_seq;

  return rnd_seq;
}

// HreRandomAlgorithm Public API
void HreRandomAlgorithm::load_goal(std::string &filename,std::vector<std::string> &xml_files)
{
  try{
    std::auto_ptr<hri_config_t> cfg(hri_config(filename,xml_schema::flags::dont_validate));
 
    xml_files.resize(NUM_CLIENTS);
    xml_files[LEDS_]=cfg->face_expression();
    xml_files[HEAD_]=cfg->head_seq();
    xml_files[LEFT_ARM_]=cfg->left_arm_seq();
    xml_files[RIGHT_ARM_]=cfg->right_arm_seq();
    xml_files[TTS_]=cfg->tts_text(); 
  }catch(const xml_schema::exception& e){
    std::ostringstream os;
    os << e;
    /* handle exceptions */
    std::cout << os.str() << std::endl;
    throw;
  }
}

void HreRandomAlgorithm::load_random_goal(std::vector<std::string> &xml_files)
{
  static int motion_count=0, speech_count=this->desired_speech_interval;
  int rnd_motion=0;
  int rnd_speech=0;

  /* randomly select a motion sequence */
  if(motion_count==0)
  {
    motion_count=this->desired_home_interval;
    try{
      std::auto_ptr<hri_config_t> cfg(hri_config(home_seq,xml_schema::flags::dont_validate));
      xml_files.resize(NUM_CLIENTS);
      xml_files[LEDS_]=std::string("");
      xml_files[HEAD_]=cfg->head_seq();
      xml_files[LEFT_ARM_]=cfg->left_arm_seq();
      xml_files[RIGHT_ARM_]=cfg->right_arm_seq();
      xml_files[TTS_]=std::string("");
    }catch(const xml_schema::exception& e){
      std::ostringstream os;
      os << e;
      /* handle exceptions */
      std::cout << os.str() << std::endl;
      throw;
    }
  }
  else
  {
    motion_count--;
    rnd_motion=random_seq();
    std::cout << "Random sequence: " << rnd_motion << std::endl;
    try{
      if(this->motion_seq_files.size()>0)
      {
        std::auto_ptr<hri_config_t> cfg(hri_config(this->motion_seq_files[rnd_motion],xml_schema::flags::dont_validate));
        xml_files.resize(NUM_CLIENTS);
        xml_files[LEDS_]=std::string("");
        xml_files[HEAD_]=cfg->head_seq();
        xml_files[LEFT_ARM_]=cfg->left_arm_seq();
        xml_files[RIGHT_ARM_]=cfg->right_arm_seq();
        xml_files[TTS_]=std::string("");
      }
      else
      {
        xml_files.resize(NUM_CLIENTS);
        xml_files[LEDS_]=std::string("");
        xml_files[HEAD_]=std::string("");
        xml_files[LEFT_ARM_]=std::string("");
        xml_files[RIGHT_ARM_]=std::string("");
        xml_files[TTS_]=std::string("");
      }
    }catch(const xml_schema::exception& e){
      std::ostringstream os;
      os << e;
      /* handle exceptions */
      std::cout << os.str() << std::endl;
      throw;
    }
  }
  /* randomly select a speech */
  if(speech_count==0)
  {
    if(this->speech_files.size()>0)
    {
      speech_count=desired_speech_interval;
      rnd_speech=rand()%this->speech_files.size();
      xml_files[TTS_]=this->speech_files[rnd_speech];
    }
  }
  else
  {
    speech_count--;
  }
}

void HreRandomAlgorithm::load_random_motion(float *x,float *y,float *theta)
{
  static int count=0;
  static float last_x;

  float dist=((float)rand()/(float)RAND_MAX)*2.0+1.0;
  float angle=2*3.14159*(float)rand()/(float)RAND_MAX;

  (*x)=dist*cos(angle);
  (*y)=dist*sin(angle);

  *theta=2*3.14159*(float)rand()/(float)RAND_MAX;

  if((last_x>0.0 && (*x)>0.0) || (last_x<0.0 && (*x)<0.0))
  {
    if(count==2)
    {
      count=0;
      (*x)=-(*x);
      last_x=(*x);
    }
    else
    {
      count++;
    }
  }
  else
    last_x=(*x);
}
