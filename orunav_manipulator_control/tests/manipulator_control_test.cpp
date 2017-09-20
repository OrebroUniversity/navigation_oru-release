#include <iostream>
#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <tinyxml.h>

double cm_to_m(double cm)
{
    return cm/100.0;
}

int main(int argc, char **argv)
{
    if(!ros::isInitialized())
    {
	ros::init(argc,argv,"manipulator_control_test");
    }
    
    ROS_INFO("This is a test for the manipulator control");
    
    std::string filename = ros::package::getPath("orunav_manipulator_control")+"/resources/output.xml";
    
    TiXmlDocument doc(filename);
    bool loadOkay = doc.LoadFile();
    if (!loadOkay)
    {
        ROS_ERROR_STREAM("Failed to load file "<<filename);
	abort();
    }
    else
    {
	std::cout<<">> Loaded: "<<filename<< std::endl <<std::endl;
    }
    
    if (doc.Type()!=TiXmlNode::TINYXML_DOCUMENT)
    {
        ROS_ERROR_STREAM("Not a TinyXML!");
        abort();
    }

    TiXmlElement* boxes=doc.FirstChildElement("boxes");
    if (boxes==NULL || boxes->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        ROS_ERROR_STREAM("Could not find element boxes into file "<<filename);
	abort();
    }
    else
    {
	TiXmlElement* box=boxes->FirstChildElement("box");

	std::cout<<"Boxes:"<<std::endl;
	
	while(box)
	{
	    std::cout<<"|"<<std::endl;
	    std::cout<<"| - Box:"<<std::endl;
	    std::cout<<"| - - sequenceNumber: "<<std::stoi(box->FirstChildElement("sequenceNumber")->GetText())<<std::endl;
	    std::cout<<"| - - item: "<<box->FirstChildElement("item")->GetText()<<std::endl;
	    std::cout<<"| - - x: "<<cm_to_m(std::stod(box->FirstChildElement("x")->GetText()))<<std::endl;
	    std::cout<<"| - - y: "<<cm_to_m(std::stod(box->FirstChildElement("y")->GetText()))<<std::endl;
	    std::cout<<"| - - z: "<<cm_to_m(std::stod(box->FirstChildElement("z")->GetText()))<<std::endl;
	    std::cout<<"| - - width: "<<cm_to_m(std::stod(box->FirstChildElement("width")->GetText()))<<std::endl;
	    std::cout<<"| - - length: "<<cm_to_m(std::stod(box->FirstChildElement("length")->GetText()))<<std::endl;
	    std::cout<<"| - - height: "<<cm_to_m(std::stod(box->FirstChildElement("height")->GetText()))<<std::endl;
	    std::cout<<"| - - cylinderFlag: "<<((box->FirstChildElement("cylinderFlag")->GetText()=="true")?1:0)<<std::endl;

	    box = box->NextSiblingElement("box");
	}
	
	std::cout<<std::endl;
    }
    
    std::cout<<std::endl;
    
    return 0;
}
