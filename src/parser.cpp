#include <parser.h>

using namespace cogimon;

gain_parser::gain_parser()
{

}

bool gain_parser::initFile(const std::string &filename)
{
    _doc.reset(new TiXmlDocument(filename));

    if(!_doc->LoadFile())
        return false;

    TiXmlHandle hDoc(_doc.get());
    TiXmlElement *pRoot, *pParm;
    TiXmlElement *ppParam, *pppParam;
    pRoot = _doc->FirstChildElement(cogimon::parsed_words::robot_tag);

    if(!pRoot)
        return false;

    pParm = pRoot->FirstChildElement(cogimon::parsed_words::rtt_gazebo_tag);
    while(pParm)
    {
         std::string kin_chain_name = pParm->Attribute(cogimon::parsed_words::reference_attribute);

         std::vector<std::string> controllers;
         cogimon::gains::PIDGains pids;
         cogimon::gains::VelPIDGains velPids;
         cogimon::gains::ImpedanceGains impedances;

         ppParam = pParm->FirstChildElement(cogimon::parsed_words::controller_tag);
         while(ppParam)
         {
            std::string controller_type = ppParam->Attribute(cogimon::parsed_words::type_attribute);
            controllers.push_back(controller_type);

            pppParam = ppParam->FirstChildElement(cogimon::parsed_words::gains_tag);
            while(pppParam)
            {
                std::string joint_name = pppParam->Attribute(cogimon::parsed_words::reference_attribute);

                if(controller_type.compare(ControlModes::JointPositionCtrl) == 0)
                {
                    std::string P = pppParam->Attribute(cogimon::parsed_words::P_attribute);
                    std::string I = pppParam->Attribute(cogimon::parsed_words::I_attribute);
                    std::string D = pppParam->Attribute(cogimon::parsed_words::D_attribute);

                    cogimon::PIDGain pid;
                    pid.joint_name = joint_name;
                    pid.P = std::atof(P.c_str());
                    pid.I = std::atof(I.c_str());
                    pid.D = std::atof(D.c_str());

                    pids.push_back(pid);
                }
                else if(controller_type.compare(ControlModes::JointVelocityCtrl) == 0)
                {
                    std::string P = pppParam->Attribute(cogimon::parsed_words::P_attribute);
                    std::string I = pppParam->Attribute(cogimon::parsed_words::I_attribute);
                    std::string D = pppParam->Attribute(cogimon::parsed_words::D_attribute);

                    cogimon::VelPIDGain velPid;
                    velPid.joint_name = joint_name;
                    velPid.P = std::atof(P.c_str());
                    velPid.I = std::atof(I.c_str());
                    velPid.D = std::atof(D.c_str());

                    velPids.push_back(velPid);
                }
                else if(controller_type.compare(ControlModes::JointImpedanceCtrl) == 0)
                {
                    std::string stiffness = pppParam->Attribute(cogimon::parsed_words::stiffness_attribute);
                    std::string damping = pppParam->Attribute(cogimon::parsed_words::damping_attribute);

                    cogimon::ImpedanceGain impedance;
                    impedance.joint_name = joint_name;
                    impedance.stiffness = std::atof(stiffness.c_str());
                    impedance.damping = std::atof(damping.c_str());

                    impedances.push_back(impedance);
                }


                pppParam = pppParam->NextSiblingElement(cogimon::parsed_words::gains_tag);
            }


            if(controller_type.compare(ControlModes::JointPositionCtrl) == 0)
                Gains.map_PIDGains[kin_chain_name] = pids;
            else if(controller_type.compare(ControlModes::JointVelocityCtrl) == 0)
                Gains.map_VelPIDGains[kin_chain_name] = velPids;
            else if(controller_type.compare(ControlModes::JointImpedanceCtrl) == 0)
                Gains.map_ImpedanceGains[kin_chain_name] = impedances;


            ppParam = ppParam->NextSiblingElement(cogimon::parsed_words::controller_tag);
         }

         if(controllers.size() > 0)
            Gains.map_controllers[kin_chain_name] = controllers;






         pParm = pParm->NextSiblingElement(cogimon::parsed_words::rtt_gazebo_tag);
    }

    return true;
}

void gain_parser::printGains()
{
    std::map<cogimon::gains::kinematic_chain, std::vector<std::string>>::iterator map_controllers_it;
    std::map<cogimon::gains::kinematic_chain, cogimon::gains::PIDGains>::iterator map_PIDGains_it;
    std::map<cogimon::gains::kinematic_chain, cogimon::gains::VelPIDGains>::iterator map_VelPIDGains_it;
    std::map<cogimon::gains::kinematic_chain, cogimon::gains::ImpedanceGains>::iterator map_ImpedanceGains_it;

    for(map_controllers_it = Gains.map_controllers.begin();
        map_controllers_it != Gains.map_controllers.end(); map_controllers_it++)
    {
        std::string kin_chain_name = map_controllers_it->first;
        std::vector<std::string> controllers = map_controllers_it->second;

        std::cout<<kin_chain_name<<": [  ";
        for(unsigned int i = 0; i < controllers.size(); ++i)
            std::cout<<controllers[i]<<"  ";
        std::cout<<"]"<<std::endl;
    }

    for(map_PIDGains_it = Gains.map_PIDGains.begin();
        map_PIDGains_it != Gains.map_PIDGains.end(); map_PIDGains_it++)
    {
        std::string kin_chain_name = map_PIDGains_it->first;
        cogimon::gains::PIDGains pids= map_PIDGains_it->second;

        std::cout<<kin_chain_name<<"  JointPositionCtrl:"<<std::endl;
        for(unsigned int i = 0; i < pids.size(); ++i)
            std::cout<<"    "<<pids[i].joint_name<<"    P: "<<pids[i].P
                    <<" I: "<<pids[i].I<<" D: "<<pids[i].D<<std::endl;
    }

    for(map_VelPIDGains_it = Gains.map_VelPIDGains.begin();
        map_VelPIDGains_it != Gains.map_VelPIDGains.end(); map_VelPIDGains_it++)
    {
        std::string kin_chain_name = map_VelPIDGains_it->first;
        cogimon::gains::VelPIDGains velPids= map_VelPIDGains_it->second;

        std::cout<<kin_chain_name<<"  JointVelocityCtrl:"<<std::endl;
        for(unsigned int i = 0; i < velPids.size(); ++i)
            std::cout<<"    "<<velPids[i].joint_name<<"    P: "<<velPids[i].P
                    <<" I: "<<velPids[i].I<<" D: "<<velPids[i].D<<std::endl;
    }

    for(map_ImpedanceGains_it = Gains.map_ImpedanceGains.begin();
        map_ImpedanceGains_it != Gains.map_ImpedanceGains.end(); map_ImpedanceGains_it++)
    {
        std::string kin_chain_name = map_ImpedanceGains_it->first;
        cogimon::gains::ImpedanceGains impedances= map_ImpedanceGains_it->second;

        std::cout<<kin_chain_name<<"  JointImpedanceCtrl:"<<std::endl;
        for(unsigned int i = 0; i < impedances.size(); ++i)
            std::cout<<"    "<<impedances[i].joint_name<<"    stiffness: "<<impedances[i].stiffness
                    <<" damping: "<<impedances[i].damping<<std::endl;
    }
}

