#include "fast_simulator/ModelParser.h"

#include "fast_simulator/Octree.h"

#include <tinyxml.h>

using namespace std;

ModelParser::ModelParser(const std::string& filename, const std::string& model_dir) : filename_(filename), model_dir_(model_dir) {

}

ModelParser::~ModelParser() {

}

vector<double> ModelParser::parseArray(const TiXmlElement* xml_elem) {
    std::string txt = xml_elem->GetText();

    vector<double> v;

    string word;
    stringstream stream(txt);
    while( getline(stream, word, ' ') ) {
        v.push_back(atof(word.c_str()));
    }

    return v;
}

Object* ModelParser::parse(const std::string& model_name, std::string& error) {
    stringstream s_error;

    //error_ = stringstream("");

    TiXmlDocument doc(filename_);
    doc.LoadFile();

    if (doc.Error()) {
        s_error << "While parsing '" << filename_ << "': " << endl << endl << doc.ErrorDesc() << " at line " << doc.ErrorRow() << ", col " << doc.ErrorCol() << endl;
        error = s_error.str();
        return 0;
    }

    const TiXmlElement* model_xml = doc.FirstChildElement("model");

    while (model_xml) {

        const char* name = model_xml->Attribute("name");
        if (name) {
            if (string(name) == model_name) {

                cout << "Parsing model '" << name << "'" << endl;

                vector<double> xyz(3, 0);
                vector<double> rpy;
                vector<double> size;

                Object* model = new Object(name);

                const TiXmlElement* shape_xml = model_xml->FirstChildElement();
                while(shape_xml) {

                    // parse properties valid for all shapes
                    const TiXmlElement* xyz_xml = shape_xml->FirstChildElement("xyz");
                    if (xyz_xml) {
                        xyz = parseArray(xyz_xml);
                    }

                    tf::Vector3 pos(xyz[0], xyz[1], xyz[2]);
                    tf::Quaternion rot(0, 0, 0, 1);

                    const TiXmlElement* rpy_xml = shape_xml->FirstChildElement("rpy");
                    if (rpy_xml) {
                        rpy = parseArray(rpy_xml);
                        if (fabs(rpy[0]) < 0.0001 && fabs(rpy[1]) < 0.0001 && fabs(rpy[2]) < 0.0001) {
                            rpy.clear();
                        } else {
                            rot.setRPY(rpy[0], rpy[1], rpy[2]);
                        }
                    }

                    const TiXmlElement* size_xml = shape_xml->FirstChildElement("size");
                    if (size_xml) {
                        size = parseArray(size_xml);
                    }

                    string shape_type = shape_xml->Value();
                    if (shape_type == "heightMap") {
                        Object* height_map = parseHeightMap(shape_xml, s_error);

                        if (height_map) {
                            height_map->setPose(pos, rot);
                            model->addChild(height_map);
                        }

                    } else if (shape_type == "box") {
                        if (!size.empty()) {

                            tf::Vector3 v_size(size[0], size[1], size[2]);

                            Object* obj = new Object();
                            if (rpy.empty()) {
                                obj->setShape(Box(pos - v_size / 2, pos + v_size / 2));
                            } else {
                                obj->setShape(Box(-v_size / 2, v_size / 2));
                                obj->setPose(pos, rot);
                            }
                            model->addChild(obj);
                        } else {
                            s_error << "In definition for model '" << name << "': shape '" << shape_type << "' has no size property" << endl;
                        }

                    } else {
                        s_error << "In definition for model '" << name << "': Unknown shape type: '" << shape_type << "'" << endl;
                    }

                    shape_xml = shape_xml->NextSiblingElement();
                }

                if (s_error.str().empty()) {
                    cout << "... Parsing successfully ..." << endl;
                    return model;
                } else {
                    return 0;
                }
            }
        } else {
            s_error << "Encountered model without 'name' attribute." << endl;
        }

        model_xml = model_xml->NextSiblingElement("model");
    }

    s_error << "No model '" << model_name << "' found." << endl;
    error = s_error.str();

    return 0;
}

Object* ModelParser::parseHeightMap(const TiXmlElement* xml_elem, stringstream& s_error) {
    const TiXmlElement* height_xml = xml_elem->FirstChildElement("height");
    double height = 0;
    if (height_xml) {
        height = atof(height_xml->GetText());
    }

    if (height <= 0) {
        s_error << "HeightMap: 'height' not or incorrectly specified." << endl;
        return 0;
    }

    const TiXmlElement* resolution_xml = xml_elem->FirstChildElement("resolution");
    double resolution = 0;
    if (resolution_xml) {
        resolution = atof(resolution_xml->GetText());
    }

    if (resolution <= 0) {
        s_error << "HeightMap: 'resolution' not or incorrectly specified." << endl;
        return 0;
    }

    const TiXmlElement* image_xml = xml_elem->FirstChildElement("image");
    if (image_xml) {
        string image_filename = image_xml->GetText();
        Object* obj = new Object();

        cout << height << ", " << resolution << endl;

        obj->setShape(Octree::fromHeightImage(model_dir_ + "/" + image_filename, height, resolution));
        return obj;
    } else {
        s_error << "HeightMap: 'image' not specified." << endl;
    }

    /*
    const TiXmlElement* topic_xml = xml_elem->FirstChildElement("topic");
    if (topic_xml) {
        string topic = topic_xml->GetText();

        ros::NodeHandle nh;
        ros::Subscriber sub_map = nh.subscribe(topic, 10, &ModelParser::callbackMap, this);
        while(ros::ok() && world_map_.data.empty()) {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            ROS_INFO("Waiting for map at topic %s", sub_map.getTopic().c_str());
        }
        ROS_INFO("Map found at topic %s", sub_map.getTopic().c_str());
        sub_map.shutdown();

        Object* root = new Object("world");
        //root->pose_ = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 3));

        createQuadTree(world_map_, 0, 0, world_map_.info.width, world_map_.info.height, height, root);

        // Object* floor = new Object();
        // floor->setShape(Box(tf::Vector3(-100, -100, -0.2), tf::Vector3(100, 100, 0)));
        // root->addChild(floor);

        return root;
    }
    */

    return 0;
}

/*
void ModelParser::createQuadTree(const nav_msgs::OccupancyGrid& map, unsigned int mx_min, unsigned int my_min,
                                                    unsigned int mx_max, unsigned int my_max, double height, Object* parent, string indent) {

    bool has_cell = false;
    for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
        for(unsigned int my = my_min; my < my_max; ++my) {
            if (map.data[map.info.width * my + mx] > 10 ) {
                has_cell = true;
            }
        }
    }

    if (!has_cell) {
        return;
    }

    Object* obj = new Object();
    tf::Vector3 min_map((double)mx_min * map.info.resolution,
                        (double)my_min * map.info.resolution, 0);
    tf::Vector3 max_map((double)mx_max * map.info.resolution,
                        (double)my_max * map.info.resolution, height);
    obj->setBoundingBox(Box(map_transform_ * min_map, map_transform_ * max_map));
    // parent->addChild(obj, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)));

    //cout << indent << mx_min << " - " << mx_max << ", " << my_min << " - " << my_max << endl;

    // cout << indent << toString(min_map) << ", " << toString(max_map) << endl;


    if (mx_max - mx_min <= 2 || my_max - my_min <= 2) {
        for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
            for(unsigned int my = my_min; my < my_max; ++my) {
                if (map.data[map.info.width * my + mx] > 10 ) {
                    tf::Vector3 pos_map((double)mx * map.info.resolution,
                                        (double)my * map.info.resolution, 0);

                    tf::Vector3 pos = map_transform_ * pos_map;

                    Object* child = new Object();
                    child->setShape(Box(pos, tf::Vector3(pos.x() + map.info.resolution,
                                                       pos.y() + map.info.resolution,
                                                       height)));
                    obj->addChild(child); //, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)));
                }
            }
        }
    } else {

        unsigned int cx = (mx_max + mx_min) / 2;
        unsigned int cy = (my_max + my_min) / 2;

        createQuadTree(map, mx_min, my_min, cx, cy, height, obj, indent + "    ");
        createQuadTree(map, cx , my_min, mx_max, cy, height, obj, indent + "    ");
        createQuadTree(map, mx_min, cy , cx, my_max, height, obj, indent + "    ");
        createQuadTree(map, cx , cy , mx_max, my_max, height, obj, indent + "    ");
    }

    parent->addChild(obj);
}

void ModelParser::callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    world_map_ = *msg;
    tf::poseMsgToTF(world_map_.info.origin, map_transform_);
    map_transform_inverse_ = map_transform_.inverse();
}
*/
