

#include "commandline.hpp"

 int main (int ac, char* av[])
 {
	int debug = 0;
	//////////////////////////////////////////////////
	std::cout<<"DEBUG STEP: " << debug <<std::endl;
	debug++;
	//////////////////////////////////////////////////
   if(ac < 2)
   {
      cout<<"No file path given"<<endl;
      return -1;
   }
   model cloud_model;
   std::vector<string> file_path;
   //////////////////////////////////////////////////
	std::cout<<"DEBUG STEP: " << debug <<std::endl;
	debug++;
	//////////////////////////////////////////////////
   for(int i = 1 ; i < ac  ; i++)
   {
      string temp = string(av[i]);
      file_path.push_back(temp);
      cout<<file_path.back()<<endl;
   }
   //////////////////////////////////////////////////
	std::cout<<"DEBUG STEP: " << debug <<std::endl;
	debug++;
	//////////////////////////////////////////////////
   cout<<"Opened..."<<cloud_model.open_files(file_path)<<"Files"<<endl;
   //////////////////////////////////////////////////
	std::cout<<"DEBUG STEP: " << debug <<std::endl;
	debug++;
	//////////////////////////////////////////////////
   

//UPDATE TEMP DIR
      for(int i = 0 ; ;i++)
   {
      string temp = "./temp/"+boost::lexical_cast<std::string>(i)+".pcd";
      boost::filesystem::path p(temp);
      if(boost::filesystem::exists(p))
      {
        boost::filesystem::remove_all(p); // delete until last MK_i file then break.
         cout<<"Deleting... "<<temp<<endl;
     }
      else
         break;
      
   }
	  //////////////////////////////////////////////////
	std::cout<<"DEBUG STEP: " << debug <<std::endl;
	debug++;
	//////////////////////////////////////////////////
   cout<<"Saving..."<<cloud_model.save_files("./temp/");
//UPDATE TEMP DIR
   //////////////////////////////////////////////////
	std::cout<<"DEBUG STEP: " << debug <<std::endl;
	debug++;
	//////////////////////////////////////////////////

   cloud_model.visualize_cloud();

   //std::cout<<"Initilized"<<std::endl;
   //model cloud_model;
   //std::vector<std::string> file_path;
   //file_path.push_back("./data/P05S10.txt");
   //file_path.push_back("./data/P05S11.txt");
   //file_path.push_back("./data/P05S13.txt.pcd");
   //
   //std::cout<<cloud_model. open_files(file_path)<<std::endl;
   //std::cout<<cloud_model. registration()<<std::endl;
   //std::cout<<cloud_model. segmentation()<<std::endl;
   //cloud_model.visualize_cloud();   
   return 0;
 }
