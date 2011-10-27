#include "iri_wiimote_teleop/wiimote_teleop_base.h"

int main(int argc,char *argv[])
{
  return iri_base_joystick::main<WiimoteTeleop>(argc, argv, "wiimote_teleop_node");
}
