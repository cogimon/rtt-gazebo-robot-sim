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
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);

    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem)
        return false;

    // save this for later
    hRoot=TiXmlHandle(pElem);



//    pElem=hRoot.FirstChild( cogimon::parsed_words::rtt_gazebo_tag ).FirstChild().Element();
//    if(!pElem)
//        return false;
//    for(pElem; pElem; pElem->NextSiblingElement())
//    {
//        std::cout<<pElem->Value()<<std::endl;
//    }

//    for( pElem; pElem; pElem=pElem->NextSiblingElement())
//    {
//        std::string pKey=pElem->Value();
//        if(pKey.compare(cogimon::parsed_words::controller_tag) == 0){
//            std::string controller_type =
//                    pElem->Attribute(cogimon::parsed_words::reference_attribute);
//            std::cout<<controller_type<<std::endl;
//        }



//    }


    return true;
}

