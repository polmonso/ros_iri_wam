#include <iostream>
#include <fstream>
#include <iri_segway_rmp_msgs/SegwayRMP400Status.h>
#include "rosbag/bag.h"
#include "ros/ros.h"

/* 
 * Quick and dirty class diagram. Strategy pattern applied.
 *
 *  [generator] ----> [Importer] abstract interface 
 *                       |
 *                       |
 *               [MatlabFileImporter]
 */

class Importer
{
    public:
        virtual bool get_item(iri_segway_rmp_msgs::SegwayRMP400Status * status, double * timestamp) = 0;
};

/*
 * This Matlab importer users the following syntax: for each line in the file,
 * space separated, should be:
 *  1. Timestamp
 *  2. Delta_t (is being ignored)
 *  3. Vx
 *  4. Vy (is being ignored)
 *  5. Vz (is being ignored)
 *  6. Wx - roll
 *  7. Wy - pitch
 *  8. Wz - yaw
 * ...  rest of the line is ignored
 */
class MatlabFileImporter : public Importer
{
    public:
        MatlabFileImporter(const std::string matlab_file);
        ~MatlabFileImporter();
        bool get_item(iri_segway_rmp_msgs::SegwayRMP400Status * status, double * timestamp);

    private:
        std::ifstream input_file_;
};

class RosbagStatusGenerator
{
    public:
        RosbagStatusGenerator(Importer * importer, const std::string output);
        ~RosbagStatusGenerator();
        void generate();

    private:
        Importer * importer_ ;
        rosbag::Bag output_bag_;
        double counter_;
        ros::Time start_time_;

        void write_to_rosbag(const iri_segway_rmp_msgs::SegwayRMP400Status status, 
                             const double timestamp);
};

MatlabFileImporter::MatlabFileImporter (const std::string matlab_file)
{
    input_file_.open(matlab_file.c_str(), std::ifstream::in);

    if (! input_file_.good())
        throw "Can not open input file";
}

MatlabFileImporter::~MatlabFileImporter()
{
    input_file_.close();
}

bool
MatlabFileImporter::get_item(iri_segway_rmp_msgs::SegwayRMP400Status * status,
                             double * timestamp)
{
    double nothing;

    iri_segway_rmp_msgs::SegwayRMP200Status status200;

    input_file_ >> * timestamp; // timestamp

    if (input_file_.fail())
        return false;

    input_file_ >> nothing;   // delta(T)
    input_file_ >> status200.right_wheel_velocity; // Vx
    input_file_ >> nothing; // Vy
    input_file_ >> nothing; // Vz
    input_file_ >> status200.roll_rate;  // roll Wx
    input_file_ >> status200.pitch_rate; // pitch Wy
    input_file_ >> status200.yaw_rate;   // yaw Wz

    status200.left_wheel_velocity = status200.right_wheel_velocity;

    status->rmp200[0] = status200;
    status->rmp200[1] = status200;

    // jump to read new line
    input_file_.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return true;
}

RosbagStatusGenerator::RosbagStatusGenerator(Importer * importer, const std::string output) :
    counter_(0.0)
{
    // This will throw an exception if it can not open
    output_bag_.open(output.c_str(), rosbag::bagmode::Write);

    importer_ = importer;
    ros::Time::init(); // needed to use ros::Time
    start_time_ = ros::Time::now();
}

void
RosbagStatusGenerator::write_to_rosbag(const iri_segway_rmp_msgs::SegwayRMP400Status status, 
                                       const double timestamp)
{
    output_bag_.write("status", start_time_ + ros::Duration(timestamp), status);
}

void
RosbagStatusGenerator::generate()
{
    iri_segway_rmp_msgs::SegwayRMP400Status status;
    double timestamp;

    while (importer_->get_item(& status, & timestamp)) {
        write_to_rosbag(status, timestamp);
        counter_++;
    }

    std::cout << " - " << counter_ << " were generated into the rosbag" << std::endl;
}


RosbagStatusGenerator::~RosbagStatusGenerator()
{
    output_bag_.close();
    delete importer_;
}

int main( int argc, const char* argv[] )
{
    if (argc != 3) {
        std::cerr << "Usage: rosbag_status_generator <input-file> <ouput-file>" << std::endl;
        return 0;
    }

    try
    {
        RosbagStatusGenerator generator(new MatlabFileImporter(argv[1]), argv[2]);
        generator.generate();
    }
    catch (std::exception & e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 1;
}
