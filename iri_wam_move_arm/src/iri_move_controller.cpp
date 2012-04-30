#include "iri_wam_move_arm/iri_move_controller.h"

IriMoveController::IriMoveController():
root_handle("/"),
private_handle("~")
{
}

IriMoveController::~IriMoveController()
{
}

void IriMoveController::receiveTraj(GoalHandleFollow gh)
{
	gh.
}
