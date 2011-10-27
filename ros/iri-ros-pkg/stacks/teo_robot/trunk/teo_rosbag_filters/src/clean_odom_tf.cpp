/* vim: set sw=4 sts=4 et foldmethod=syntax : */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <boost/foreach.hpp>

#include <tf/tfMessage.h>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: clean_odom_tf <input.bag>" << std::endl;
        exit(-1);
    }

    rosbag::Bag bag;
    rosbag::Bag output("output.bag", rosbag::bagmode::Write);

    int i = 0;
    bag.open(argv[1], rosbag::bagmode::Read);

    rosbag::View view(bag);

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        if (m.getTopic() == "/tf")
        {
            tf::tfMessage::ConstPtr tf = m.instantiate<tf::tfMessage>();
            if (tf->transforms[0].header.frame_id == "/odom")
            {
                i++;
                continue;
            }
        }
        output.write(m.getTopic(),m.getTime(),m);
    }

    bag.close();

    std::cout << "Total number of tfs removed: " << i << std::endl;
    return 1;
}
