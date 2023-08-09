#include "fast_simulator/ModelParser.h"

//#include "fast_simulator/Octree.h"

#include <geolib/HeightMap.h>

// for loading images
#include <opencv2/highgui/highgui.hpp>

#include <tinyxml2.h>

#include <ed/update_request.h>
#include <geolib/shapes.h>

ModelParser::ModelParser(const std::string& filename, const std::string& model_dir) : filename_(filename), model_dir_(model_dir) {

}

ModelParser::~ModelParser() {

}

std::vector<double> ModelParser::parseArray(const tinyxml2::XMLElement* xml_elem) {
    std::string txt = xml_elem->GetText();

    std::vector<double> v;

    std::string word;
    std::stringstream stream(txt);
    while( getline(stream, word, ' ') ) {
        v.push_back(atof(word.c_str()));
    }

    return v;
}

// ----------------------------------------------------------------------------------------------------

//void addObjectRecursive(const ed::models::NewEntityConstPtr& e, Object* obj)
//{
//    if (e->shape)
//        obj->setShape(*e->shape);

//    for(std::vector<ed::models::NewEntityPtr>::const_iterator it = e->children.begin(); it != e->children.end(); ++it)
//    {
//        const ed::models::NewEntityPtr& e_child = *it;

//        Object* child = new Object(e_child->type, e_child->id);
//        addObjectRecursive(e_child, child);

//        obj->addChild(child, e_child->pose);
//    }
//}

// ----------------------------------------------------------------------------------------------------

Object* ModelParser::parse(const std::string& model_name, const std::string& id, std::string& error)
{
    if (ed_model_loader_.exists(model_name))
    {
        std::stringstream s_error_ed;
        ed::UpdateRequest req;
        if (!ed_model_loader_.create(id, model_name, req, s_error_ed, true))
        {
            error = s_error_ed.str();
            return 0;
        }

        Object* obj_root = new Object(model_name, id);

        for(std::map<ed::UUID, geo::Pose3D>::const_iterator it = req.poses.begin(); it != req.poses.end(); ++it)
        {
            const ed::UUID& ed_id = it->first;
            const geo::Pose3D& pose = it->second;

            geo::ShapeConstPtr shape;
            std::map<ed::UUID, geo::ShapeConstPtr>::const_iterator it_shape = req.visuals.find(ed_id);
            if (it_shape != req.visuals.end())
                shape = it_shape->second;

            if (it_shape != req.visuals.end())
            {
                if (ed_id.str() == id)
                {
                    obj_root->setPose(pose);
                    if (shape)
                        obj_root->setShape(*shape);
                }
                else
                {
                    Object* obj = new Object(model_name, ed_id.str());

                    if (shape)
                        obj->setShape(*shape);

                    obj_root->addChild(obj, pose);
                }
            }
        }

        std::cout << "Model '" << model_name << "' loaded from ed_object_models" << std::endl;
        return obj_root;
    }

    std::stringstream s_error;

    //error_ = stringstream("");

    tinyxml2::XMLDocument doc;
    doc.LoadFile(filename_.c_str());

    if (doc.Error()) {
        s_error << "While parsing '" << filename_ << "': " << std::endl << std::endl << doc.ErrorStr() << " at line " << doc.ErrorLineNum() << std::endl;
        error = s_error.str();
        return 0;
    }

    const tinyxml2::XMLElement* model_xml = doc.FirstChildElement("model");

    while (model_xml) {

        const char* name = model_xml->Attribute("name");
        if (name) {
            if (std::string(name) == model_name) {

                std::cout << "Parsing model '" << name << "'" << std::endl;

                std::vector<double> xyz(3, 0);
                std::vector<double> rpy;
                std::vector<double> size;

                Object* model = new Object(name);

                const tinyxml2::XMLElement* shape_xml = model_xml->FirstChildElement();
                while(shape_xml) {

                    // parse properties valid for all shapes
                    const tinyxml2::XMLElement* xyz_xml = shape_xml->FirstChildElement("xyz");
                    if (xyz_xml) {
                        xyz = parseArray(xyz_xml);
                    }

                    geo::Vector3 pos(xyz[0], xyz[1], xyz[2]);
                    geo::Matrix3 rot = geo::Matrix3::identity();

                    const tinyxml2::XMLElement* rpy_xml = shape_xml->FirstChildElement("rpy");
                    if (rpy_xml) {
                        rpy = parseArray(rpy_xml);
                        if (fabs(rpy[0]) < 0.0001 && fabs(rpy[1]) < 0.0001 && fabs(rpy[2]) < 0.0001) {
                            rpy.clear();
                        } else {
                            rot.setRPY(rpy[0], rpy[1], rpy[2]);
                        }
                    }

                    const tinyxml2::XMLElement* size_xml = shape_xml->FirstChildElement("size");
                    if (size_xml) {
                        size = parseArray(size_xml);
                    }

                    std::string shape_type = shape_xml->Value();
                    if (shape_type == "heightMap") {
                        Object* height_map = parseHeightMap(shape_xml, s_error);

                        if (height_map) {
                            height_map->setPose(geo::Transform(rot, pos));
                            model->addChild(height_map);
                        }

                    } else if (shape_type == "box") {
                        if (!size.empty()) {

                            geo::Vector3 v_size(size[0], size[1], size[2]);

                            Object* obj = new Object();
                            if (rpy.empty()) {
                                obj->setShape(geo::Box(pos - v_size / 2, pos + v_size / 2));
                            } else {
                                obj->setShape(geo::Box(-v_size / 2, v_size / 2));
                                obj->setPose(geo::Transform(rot, pos));
                            }
                            model->addChild(obj);
                        } else {
                            s_error << "In definition for model '" << name << "': shape '" << shape_type << "' has no size property" << std::endl;
                        }
                    } else if (shape_type == "cylinder") {
                        if (!size.empty()) {

                            geo::Shape shape;
                            geo::createCylinder(shape, size[0] / 2, size[2]);

                            Object* obj = new Object();
                            if (rpy.empty()) {
                                obj->setShape(shape);
                            } else {
                                obj->setShape(shape);
                                obj->setPose(geo::Transform(rot, pos));
                            }
                            model->addChild(obj);
                        } else {
                            s_error << "In definition for model '" << name << "': shape '" << shape_type << "' has no size property" << std::endl;
                        }
                    } else {
                        s_error << "In definition for model '" << name << "': Unknown shape type: '" << shape_type << "'" << std::endl;
                    }

                    shape_xml = shape_xml->NextSiblingElement();
                }

                error = s_error.str();
                if (s_error.str().empty()) {
                    std::cout << "... Parsing successfully ..." << std::endl;
                    return model;
                } else {
                    return 0;
                }
            }
        } else {
            s_error << "Encountered model without 'name' attribute." << std::endl;
        }

        model_xml = model_xml->NextSiblingElement("model");
    }

    //s_error << "No model '" << model_name << "' found." << endl;
    error = s_error.str();

    return 0;
}

Object* ModelParser::parseHeightMap(const tinyxml2::XMLElement* xml_elem, std::stringstream& s_error) {
    const tinyxml2::XMLElement* height_xml = xml_elem->FirstChildElement("height");
    double height = 0;
    if (height_xml) {
        height = atof(height_xml->GetText());
    }

    if (height <= 0) {
        s_error << "HeightMap: 'height' not or incorrectly specified." << std::endl;
        return 0;
    }

    const tinyxml2::XMLElement* resolution_xml = xml_elem->FirstChildElement("resolution");
    double resolution = 0;
    if (resolution_xml) {
        resolution = atof(resolution_xml->GetText());
    }

    if (resolution <= 0) {
        s_error << "HeightMap: 'resolution' not or incorrectly specified." << std::endl;
        return 0;
    }

    const tinyxml2::XMLElement* image_xml = xml_elem->FirstChildElement("image");
    if (image_xml) {
        std::string image_filename = image_xml->GetText();
        Object* obj = new Object();

        geo::HeightMap hmap = getHeightMapFromImage(model_dir_ + "/" + image_filename, height, resolution);
        obj->setShape(hmap);

        return obj;
    } else {
        s_error << "HeightMap: 'image' not specified." << std::endl;
    }

    return 0;
}

geo::HeightMap ModelParser::getHeightMapFromImage(const std::string& image_filename, double height, double resolution) {

    cv::Mat image = cv::imread(image_filename, cv::IMREAD_GRAYSCALE);   // Read the file

    std::vector<std::vector<double> > map;

    if (image.data ) {

        map.resize(image.cols);

        for(int x = 0; x < image.cols; ++x) {
            map[x].resize(image.rows);
            for(int y = 0; y < image.rows; ++y) {
                map[x][image.rows - y - 1] = height - (double)image.at<unsigned char>(y, x) / 255 * height;
            }
        }

        std::cout << "Loaded height map " << image_filename << std::endl;

    } else {
        std::cout << "Could not load height map " << image_filename << std::endl;
    }

    return geo::HeightMap::fromGrid(map, resolution);
}
